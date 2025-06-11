#include <Arduino.h>
#include "teensy_comm.h"
#include "step.h"
#include "Encoder.h"
#include <TimerOne.h>
#include <imxrt.h> // Make sure Teensy core libraries are included

#define enc_pin_1A 14
#define enc_pin_2A 16
#define enc_pin_3A 18
#define enc_pin_4A 21
#define enc_pin_5A 23
#define enc_pin_6A 25
#define enc_pin_7A 0
#define enc_pin_8A 0

#define enc_pin_1B 15
#define enc_pin_2B 17
#define enc_pin_3B 19
#define enc_pin_4B 20
#define enc_pin_5B 22
#define enc_pin_6B 24
#define enc_pin_7B 0
#define enc_pin_8B 0

#define step_pin_1 3 
#define step_pin_2 5 
#define step_pin_3 7
#define step_pin_4 9
#define step_pin_5 11
#define step_pin_6 13

#define step_pin_extend_launcher 28
#define step_pin_launch_ball 37

#define dir_pin_1 2
#define dir_pin_2 4
#define dir_pin_3 6
#define dir_pin_4 8
#define dir_pin_5 10
#define dir_pin_6 12

#define dir_pin_extend_launcher 29
#define dir_pin_launch_ball 36

//    Zeroing pin will be in pins 39,40, and 41
#define lim_switch_a 39
#define lim_switch_b 40
#define lim_switch_c 41

#define zeroing_timout_value 2000


int32_t position_cmds[NUM_JOINTS];
int32_t encoder_positions[NUM_JOINTS];
int prevEncoderPos = 0;

static teensy_status_t tnsy_sts = {0};
static teensy_command_t tnsy_cmd = {0};
static teensy_zero_t tnsy_zero = {0};
static teensy_header_t tnsy_hdr = {0};

typedef struct TaskScheduler 
{
  bool state;
  volatile int elapsedTime;
  int period;
  void (*function)();
} scheduledTasks;

scheduledTasks tasks[NUM_JOINTS];

// how many ISR cycles to stay on
const int m1_OnTime = 1;
const int m2_OnTime = 1;
const int m3_OnTime = 1;
const int m4_OnTime = 1;
const int m5_OnTime = 1;
const int m6_OnTime = 1;
const int m7_OnTime = 1;
const int m8_OnTime = 1;

//period between the functions in
const int m1_period = 0;
const int m2_period = 0;
const int m3_period = 0;
const int m4_period = 0;
const int m5_period = 0;
const int m6_period = 0;
const int m7_period = 0;
const int m8_period = 0;

double velocities[NUM_JOINTS];

uint32_t encoderValues[NUM_JOINTS];

Stepper myStepper[] = 
{ 
  Stepper(step_pin_1, dir_pin_1, 1000,  1, 1),
  Stepper(step_pin_2, dir_pin_2, 300,   2, 1),
  Stepper(step_pin_3, dir_pin_3, 1000,  3, 1),
  Stepper(step_pin_4, dir_pin_4, 300,   4, 1),
  Stepper(step_pin_5, dir_pin_5, 1000,  5, 1),
  Stepper(step_pin_6, dir_pin_6, 300,   6, 0),
  Stepper(step_pin_extend_launcher, dir_pin_extend_launcher,  1, 7, 0),
  Stepper(step_pin_launch_ball, dir_pin_launch_ball,          1, 8, 0)
};


Encoder myEncoder[] = { 
  Encoder(enc_pin_1A, enc_pin_1B),
  Encoder(enc_pin_2A, enc_pin_2B),
  Encoder(enc_pin_3A, enc_pin_3B),
  Encoder(enc_pin_4A, enc_pin_4B),
  Encoder(enc_pin_5A, enc_pin_5B),
  Encoder(enc_pin_6A, enc_pin_6B)
  // Encoder(enc_pin_7A, enc_pin_7B)};
  // Encoder(enc_pin_8A, enc_pin_8B)};
};

//****************************************    High Level Code       ****************************************


/**
 * Calls setup and loop 
 */
int main(void) 
{
  setup();
  //motor_test(6, -10);   // (motor_choice, speed)  // motor_choice is 1-8, not 0-7
  //zeroing();
  new_pos(1, 100000);
  loop();
}


/**
 * S E T U P
 * 
 * Initializes the LED, initializes the task scheduler, initializes the timer 
 * interrupt with a 10 microsecond period. Starts the Jetson serial comm link
 * with a 8N1 communication scheme and a baud rate of 115200.
 */
void setup(void) {
  
  Serial.begin(115200);
  Serial.println("Setup will begin ");

  noInterrupts();
  pinMode(13, OUTPUT);
  initEncoders();

  for (int ii = 0; ii < NUM_JOINTS; ++ii) 
  {
    tasks[ii].state = false;
    tasks[ii].elapsedTime = 0;
    tasks[ii].period = 1000000;
    
    // dont init the last two encoders
    if (myStepper[ii].motor_id == 7 || myStepper[ii].motor_id == 8)   // if (myStepper[ii].closed_loop)
    {
      position_cmds[ii] = 0;
    }
  }       

  Timer1.initialize(10);             // interrupt every 10 us
  Timer1.attachInterrupt(motorISR);  // motorISR is the ISR
  Timer1.start();
  
  Serial1.begin(115200, SERIAL_8N1);
  //Serial.begin(115200);

  Serial.println("Setup Complete");

  interrupts();
}


/**
 * L O O P
 * 
 * Receives serial commands from Jetson on serial port 1, updates the motor 
 * position setpoints, reads encoder positions, and sends status serial message
 * back to Jetson.
 */

void loop(void) 
{
  while (1)
  {
    // Serial Receive
    receive_command();

    // Read Encoders
    //read_encoders();

    motor_task();

    // Serial Send
    send_status();
/*
    for (int ii = 0; ii < NUM_JOINTS; ++ii) 
    {
      if (myStepper[ii].closed_loop)  // closed loop
      {
        tasks[ii].period = myStepper[ii].newFrequency(myEncoder[ii].read(), position_cmds[ii]);
      //Serial.printf("motor %d  period = %lu µs\n", ii, tasks[ii].period);
      }
      else if (ii == 6)  // speicial command for joint 7, that allows it to get to the desired position, which is outside the current limit of the communcated values
      {
        tasks[ii].period = myStepper[ii].newFrequency(myStepper[ii].current_angle, position_cmds[ii] * 10);
      }
      else  // open loop
      {
        tasks[ii].period = myStepper[ii].newFrequency(myStepper[ii].current_angle, position_cmds[ii]);
      }
    }
    */

    print_encoder_values();
    //print_target_values();

    delay(10);
  
  }
}


//****************************************    Low Level Code       ****************************************

/**
 * Reads the current position of all encoders into myStepper
 */
void read_encoders() 
{ 
  for(int ii = 0; ii < NUM_JOINTS; ++ii) 
  { 
    if (myStepper[ii].closed_loop)
      encoder_positions[ii] = myEncoder[ii].read();

    if (!myStepper[ii].closed_loop)
      encoder_positions[ii] = myStepper[ii].current_angle;  
  }
}

void motor_task()
{
  for (int ii = 0; ii < NUM_JOINTS; ++ii) 
  {
    encoder_positions[ii] = get_position(ii);
    tasks[ii].period = myStepper[ii].newFrequency(get_position(ii), position_cmds[ii]);
  }
}

int get_position(int motor)
{
  if (myStepper[motor].closed_loop)   // closed loop
    return position_cmds[motor];
  else if (motor == 6)                // speicial command for joint 7, that allows it to get to the desired position, which is outside the current limit of the communcated values
    return position_cmds[motor] * 10;
  else                                // open loop
    return position_cmds[motor];
}



/**
 * Initializes all encoders to zero 
 */
void initEncoders() 
{
  for (int kk = 0; kk < NUM_JOINTS; ++kk) 
  {
    if (myStepper[kk].closed_loop)
      myEncoder[kk].write(0);

    if (!myStepper[kk].closed_loop)
      encoder_positions[kk] = 0;  
  }
}

/**
 * Receives bytes on serial port and pareses messages. 
 * 
 * @return int - 0 success, -1 too many bytes read, -2 message parse error,
 *               OTHER: message type
 */
int receive_command() 
{
  uint8_t buffer[1024] = { 0 };
  uint8_t bytes_received = 0;
  int ret = 0;

  while (Serial1.available()) 
  {

    if (bytes_received >= sizeof(buffer)) 
    {
      Serial.println("Error: Buffer overflow!");
      return -1;
    }

    Serial1.readBytes(&buffer[bytes_received], 1);
    bytes_received++;

    // Only parse when a full message is received
    if (bytes_received == sizeof(tnsy_cmd))
    {
      ret = parse_message(&buffer[0], bytes_received);

      if (ret == 1) // TODO: make sure all messages are handled
      {
        for (int j = 0; j < NUM_JOINTS; ++j) 
        {
          position_cmds[j] = tnsy_cmd.setpoint_position[j];
        }
        return 1;
      } 
      else if (ret == 2)
      {
        // could put Zeroing function call
        return 2;
      }
      else 
      {
        return -2;
      }
    }

  }
  return 0;
}


/**
 * @brief - parses incoming serial messages 
 * 
 * @return int - -3 unknown message type, -2 did not pass header, -1 did not 
 *               pass CRC, OTHER: message type.
 */
int parse_message(const uint8_t* buf, int size) 
{

  uint16_t calc_crc = 0;
  uint16_t rec_crc = 0;
  uint16_t hdr_chk = 0;

  memcpy(&hdr_chk, buf, sizeof(hdr_chk));

  if (hdr_chk == 0x5555) 
  {
    calc_crc = crc16_ccitt(buf, size - 2);
    memcpy(&rec_crc, buf + size - 2, sizeof(rec_crc));

    if (calc_crc == rec_crc)
    {
      memcpy(&tnsy_hdr, buf, size - 2);

      // Added message type support. TODO: add this to the jetson
      switch (tnsy_hdr.type)
      {
        case 0:
          // TODO: update Jetson with correct message type for generic command message.
          memcpy(&tnsy_cmd, buf, size - 2);
          return 0;
          break;

        case 1:
          memcpy(&tnsy_cmd, buf, size - 2);
          return 1;
          break;

        case 2:
          memcpy(&tnsy_zero, buf, size - 2);
          return 2;
          break;

        default:
          Serial.print("Error with message parsing");
          return -3;
          break;
      }

    } 
    else 
    {
      return -1;
    }
  } 
  else 
  {
    return -2;
  }
}


/**
 * @brief - constructs packet with latest encoder data to send to jetson. Sends 
 *          packet once the construction is complete. Message structure is 
 *          defined in teensy_comm.h.
 * 
 * @return int - 0 success, 1 fail
 */
int send_status (void) 
{
  uint8_t buffer[1024] = {0};

  tnsy_sts.hdr.header = 0x5555;
  tnsy_sts.hdr.seq++;
  tnsy_sts.hdr.len = sizeof(tnsy_sts);
  tnsy_sts.hdr.type = 0;

  for (int ii = 0; ii < NUM_JOINTS; ii++) 
  {
    tnsy_sts.encoder[ii] = encoder_positions[ii];
  }

  memcpy(&buffer[0], &tnsy_sts, sizeof(tnsy_sts) - 2);
  tnsy_sts.crc = crc16_ccitt(&buffer[0], sizeof(tnsy_sts) - 2);

  memcpy(&buffer[0], &tnsy_sts, sizeof(tnsy_sts));

  Serial1.write(buffer, sizeof(tnsy_sts));

  return 0;
}


/** 
 * Interrupt Service Routine for stepping the motors at the correct rate and to
 * the correct position.
 */
void motorISR(void) 
{
  for (int ii = 0; ii < NUM_JOINTS; ++ii) 
  {
    if (tasks[ii].period == 0) 
    {
      continue;          
    }

    //increase the elapsed time since the last time the function was called
    tasks[ii].elapsedTime += 10;

    if (tasks[ii].elapsedTime >= tasks[ii].period) 
    {
      myStepper[ii].step();       //call the step function
      tasks[ii].elapsedTime = 0;  //reset the elapsed time
      
      if (myStepper[ii].motor_id == 7) // speicial *10 for special joint 7 funcitnallity, ask Elias, this is his stuipid work around, cuz he's to lazy to fix it as of right now, what a bum
      {
        if( (position_cmds[ii] * 10) - myStepper[ii].current_angle > 0.5)  // will have an error buffer of .5 in either direction
          {
            myStepper[ii].current_angle++;
          }
        else if((position_cmds[ii] * 10) - myStepper[ii].current_angle < -0.5)
        {
          myStepper[ii].current_angle--;
        }
      }
      else if (!myStepper[ii].closed_loop) // ajust angle of closed loop joints
      {
        if(position_cmds[ii] - myStepper[ii].current_angle > 0.5)  // will have an error buffer of .5 in either direction
          {
            myStepper[ii].current_angle++;
          }
        else if(position_cmds[ii] - myStepper[ii].current_angle < -0.5)
        {
          myStepper[ii].current_angle--;
        }
      }
    }
  }
}

//****************************************    Zeroing       ****************************************
/**
*
*
 */

void zeroing(void)
{                           //   How to move joint: position_cmds[motor] = pos;
  int motor_speed = -5;

  zeroing_loop(1, lim_switch_a, motor_speed, zeroing_timout_value); // Input motor number, not motor index
  zeroing_loop(3, lim_switch_b, motor_speed, zeroing_timout_value);
  zeroing_loop(5, lim_switch_c, motor_speed, zeroing_timout_value);
}

void zeroing_loop(int motor, int limswitch, int motors_speed, int timeout)
{
  int ii;

  --motor;  // this makes it so that motor number is input not motor index

  for (ii = 0; ii < zeroing_timout_value; ++ii)
  {
    //tasks[motor].period = myStepper[motor].newFrequency(myEncoder[motor].read(), myEncoder[motor].read() + motors_speed);
    position_cmds[motor] = position_cmds[motor] + motors_speed;
    motor_task();

    print_lim_switches(motor, false);
    print_encoder_values();

    delay(5);

    if (readGPIOFast(limswitch))
    {
      motor_reset();
      return;
    }
  }

  print_lim_switches(motor, true);
  motor_reset();
}

void motor_reset()
{
  for (int jj = 0; jj < NUM_JOINTS; ++jj) 
  {
    position_cmds[jj] = 0;

    if (myStepper[jj].closed_loop)
    {
      myEncoder[jj].write(0);
      tasks[jj].period = myStepper[jj].newFrequency(myEncoder[jj].read(), position_cmds[jj]);
    }

    if (!myStepper[jj].closed_loop)
    {
      encoder_positions[jj] = 0;  
      tasks[jj].period = myStepper[jj].newFrequency(0, 0);
    }
  }  /// Keep all motors where they are
}

void new_pos ( int motor, int goal_pos ) // input motor NUMBER not motor INDEX
{
  int cur_pos = 0;

  --motor;
  position_cmds[motor] = goal_pos;

  for (int i = 0; i < zeroing_timout_value; ++i)
  {
    motor_task();
    cur_pos = get_position(motor);

      if (abs(cur_pos - goal_pos) < 2) // will end function off as soon as it's within an error radius of 2
      {
        return;
      }  
  }
  Serial.println("AHAHAHA, THIS MOVEMNET FAILED YOU DUMB NUT");
}

bool readGPIOFast(int pin) 
{
  return *(portInputRegister(pin)) & digitalPinToBitMask(pin);
}


//****************************************    Test Functions       ****************************************
void motor_test(int tested_motor, int speed)
{
  Serial.print("Testing Motor ");
  Serial.print(tested_motor);
  Serial.println(" ");

  int i;

  --tested_motor;   // this is so that the above comment is true

  while(1)
  {

    tasks[tested_motor].period = myStepper[tested_motor].newFrequency(0, speed);

    for (i = 0; i < NUM_JOINTS - NUM_EJCT_JOINTS; ++i) 
    {
      if (i != tested_motor)
      {
        tasks[i].period = myStepper[i].newFrequency(myEncoder[i].read(), 0);
      } 
    } /// Keep other motors where they are
    for (i = NUM_JOINTS - NUM_EJCT_JOINTS; i < NUM_EJCT_JOINTS; ++i)
    {
      if (i != tested_motor)
      {
        tasks[i].period = myStepper[i].newFrequency(0, 0);
      }
    }

    delay(5);
  }
}


//****************************************    Print Functions       ****************************************
void print_lim_switches (int motor_in, int failure)
{
  if( failure )
  {
    Serial.print("Zeroing failed on motor ");
    Serial.print(motor_in);
    Serial.println(" ");
  }
  else
  {
    Serial.print("Check: ");
    Serial.print(motor_in + 1);
    Serial.println(" ");
  }
}

void print_encoder_values (void)
{
  // Encoder readout  
  for ( int i = 0; i < NUM_JOINTS; ++i)
  {
    Serial.print(encoder_positions[i]);
    Serial.print(" ");
  }
  Serial.println(" ");
}

void print_target_values (void)
{
  for ( int i = 0; i < NUM_JOINTS; ++i)
  {
    Serial.print(position_cmds[i]);
    Serial.print(" ");
  }
  Serial.println(" ");
}

/**

If the permission port permission error occurs, use the following commands on Jetson

sudo usermod -a -G tty <username>
sudo usermod -a -G dialout <username>
sudo usermod -a -G sudo <username>

sudo chmod g+rw /dev/ttyTHS1


*/
