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

#define step_pin_1 0 //3
#define step_pin_2 0 //5
#define step_pin_3 7
#define step_pin_4 9
#define step_pin_5 11
#define step_pin_6 13

#define step_pin_extend_launcher 3 //36
#define step_pin_launch_ball 5 //28

#define dir_pin_1 0 //2
#define dir_pin_2 0 //4
#define dir_pin_3 6
#define dir_pin_4 8
#define dir_pin_5 10
#define dir_pin_6 12

#define dir_pin_extend_launcher 2 //37
#define dir_pin_launch_ball 4 //29

//    Zeroing pin will be in pins 39,40, and 41
#define lim_switch_a 39
#define lim_switch_b 40
#define lim_switch_c 41


int32_t position_cmds[NUM_JOINTS];
int32_t encoder_positions[NUM_JOINTS];
int prevEncoderPos = 0;

// For Test Seial Communication
static teensy_status_t tnsy_sts = { 0 };
static teensy_command_t tnsy_cmd = { 0 };

typedef struct TaskScheduler {
  bool state;
  volatile int elapsedTime;
  int period;
  void (*function)();
} scheduledTasks;

scheduledTasks tasks[NUM_JOINTS];

//void motorISR(void);

//int tasksPerPeriod = 10;
//int tasksNum = 6;

// how many ISR cycles to stay on
const int m1_OnTime = 1;
const int m2_OnTime = 1;
const int m3_OnTime = 1;
const int m4_OnTime = 1;
const int m5_OnTime = 1;
const int m6_OnTime = 1;

//period between the functions in
const int m1_period = 0;
const int m2_period = 0;
const int m3_period = 0;
const int m4_period = 0;
const int m5_period = 0;
const int m6_period = 0;
const int m7_period = 0;

double velocities[NUM_JOINTS];

uint32_t encoderValues[NUM_JOINTS];

Stepper myStepper[] = { 
  Stepper(step_pin_1, dir_pin_1, 1000, 1),
  Stepper(step_pin_2, dir_pin_2, 1000, 2),
  Stepper(step_pin_3, dir_pin_3, 300, 3),
  Stepper(step_pin_4, dir_pin_4, 300, 4),
  Stepper(step_pin_5, dir_pin_5, 300, 5),
  Stepper(step_pin_6, dir_pin_6, 300, 6)
  //Stepper(step_pin_7, dir_pin_7, enc_pin_7A, enc_pin_7B, 1000, 7)};
//Stepper(step_pin_8, dir_pin_8, enc_pin_8A, enc_pin_8B, 1000, 8)};
};


Encoder myEncoder[] = { 
  Encoder(enc_pin_1A, enc_pin_1B),
  Encoder(enc_pin_2A, enc_pin_2B),
  Encoder(enc_pin_3A, enc_pin_3B),
  Encoder(enc_pin_4A, enc_pin_4B),
  Encoder(enc_pin_5A, enc_pin_5B),
  Encoder(enc_pin_6A, enc_pin_6B)
//Encoder(enc_pin_7A, enc_pin_7B)};
//Encoder(enc_pin_8A, enc_pin_8B)};
};


int velocity[6];        //            HEY RILEY SHOULD THIS BE NUM_JOINTS INSTEAD OF 6?
//Encoder(enc_pin_8A, enc_pin_8B)};


//****************************************    High Level Code       ****************************************



int main(void) 
{

  setup();
  //zeroing();
  loop();

}


void setup(void) {
  noInterrupts();
  pinMode(13, OUTPUT);
  initEncoders();
  
  //pinMode(13, OUTPUT);  // led pin to output      THIS WAS ALREADY HAPPENING



  for (int ii = 0; ii < NUM_JOINTS; ++ii) {
    tasks[ii].state = false;
    tasks[ii].elapsedTime = 0;
    tasks[ii].period = 1'000'000UL;
    position_cmds[ii] = myEncoder[ii].read();
  }   // This used to not be a for loop


  Timer1.initialize(10);             // interrupt every 10 us
  Timer1.attachInterrupt(motorISR);  // motorISR is the ISR
  Timer1.start();
  
  interrupts();

  //setup_comm();
  Serial1.begin(115200, SERIAL_8N1);
  Serial.begin(115200);

  Serial.println("Setup Complete");
}


void loop(void) {

  while (1)
  {
    // Serial Recive
    recieve_command();

    // Read Encodors
    sendEncoderValues();

    // Serial Send
    send_status();


    //    sendEncoderValues();

    for ( int ii = 0; ii < NUM_JOINTS; ++ii) {
      tasks[ii].period = myStepper[ii].newFrequency(myEncoder[ii].read(), position_cmds[ii]);
      //Serial.printf("motor %d  period = %lu µs\n", ii, tasks[ii].period);
    }  // set the period of each motor based on the velocities recived from the jetson 



    print_econder_values();
    //print_target_values();


    delay(10);
  
  }
}



//****************************************    Low Level Code       ****************************************

/*
**************************************** -=+ Encoder Transmit Sequence +=- *****************************************/
// - Populates Encoder values
void sendEncoderValues() {
  
  for(int ii = 0; ii < NUM_JOINTS; ++ii) 
  { 
    encoder_positions[ii] = myEncoder[ii].read();

  }
}

/***************************************** -=+ Initialize Encoders +=- *****************************************/
void initEncoders() {
  for (int kk = 0; kk < NUM_JOINTS; ++kk) {
    myEncoder[kk].write(0);
  }
}

// Serial Function
int recieve_command() {
  uint8_t buffer[1024] = { 0 };
  uint8_t bytes_recieved = 0;

  while (Serial1.available()) 
  {

    if (bytes_recieved >= sizeof(buffer)) {
      Serial.println("Error: Buffer overflow!");
      return -1;
    }

    Serial1.readBytes(&buffer[bytes_recieved], 1);
    bytes_recieved++;

    // Only parse when a full message is received
    if (bytes_recieved == sizeof(tnsy_cmd))
    {
      //Serial.println("full message");


      
      if (parse_message(&buffer[0], bytes_recieved) == 0) 
      {
        for (int j = 0; j < NUM_JOINTS; ++j) 
        {
          position_cmds[j] = tnsy_cmd.setpoint_position[j];
        }

      } else {
        return 2;
      }
    }
          
          /*
          
      digitalWrite(4, HIGH);      // Why is this controlling pin 4????
      delay(500);
      digitalWrite(4, LOW);

      if (parse_message(&buffer[0], bytes_recieved) == 0) 
      {
        digitalWrite(4, HIGH);
        delay(1000);
        digitalWrite(4, LOW);

        for (int j = 0; j < NUM_JOINTS; ++j) 
        {
          position_cmds[j] = tnsy_cmd.setpoint_position[j];
        }

        return 0;
      } else {
        Serial.println("Error: Invalid message received.");
        digitalWrite(4, HIGH);
        delay(100);
        digitalWrite(4, LOW);

        return 2;
      }
    }   */
        
  }
  return 0;
}

// Serial Funtion
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
    if (calc_crc == rec_crc) {
      //Serial.println(" memory copy and calc=rec");

      memcpy(&tnsy_cmd, buf, size - 2);

      return 0;
    } else {
      return 2;
    }
  } else {
    // ROS_WARN("did not pass header");
    return 1;
  }
}


// Serial Function
int send_status() 
{
  uint8_t buffer[256] = { 0 };

  if (sizeof(tnsy_sts) > sizeof(buffer)) 
  {
    Serial.println("Error: tnsy_sts too large");
    return 1;
  }

  tnsy_sts.hdr.header = 0x5555;
  tnsy_sts.hdr.seq++;
  tnsy_sts.hdr.len = sizeof(tnsy_sts);

  for (int ii = 0; ii < NUM_JOINTS; ii++) {
    //tnsy_sts.encoder[ii] = tnsy_cmd.setpoint_position[ii];
    tnsy_sts.encoder[ii] = encoder_positions[ii];
  }

  memcpy(&buffer[0], &tnsy_sts, sizeof(tnsy_sts) - 2);
  tnsy_sts.crc = crc16_ccitt(&buffer[0], sizeof(tnsy_sts) - 2);

  memcpy(&buffer[0], &tnsy_sts, sizeof(tnsy_sts));

  Serial1.write(buffer, sizeof(tnsy_sts));

  return 0;
}


bool stepToPos(int dirPin, int stepPin, int goal_step_num, int direction, int delayTime, int current_step_num ) 
{ 
    // Set direction
  digitalWrite(dirPin, direction == 0 ? HIGH : LOW);
    
    // Step the motor the specified number of steps
  if( current_step_num < goal_step_num ) 
  { 
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(delayTime); // Control speed
    digitalWrite(stepPin, LOW);
    delayMicroseconds(delayTime);
  }
  return (current_step_num <  goal_step_num ); // will keep going if the curr pos is not yet at goal
}


void motorISR(void) {
  for (int ii = 0; ii < NUM_JOINTS; ++ii) {

    if (tasks[ii].period == 0) continue;          // skip uninitialised motor
    tasks[ii].elapsedTime += 10;                  //increase the elapsed time since the last time the function was called
    if (tasks[ii].elapsedTime >= tasks[ii].period) 
    {
      myStepper[ii].step();       //call the step function
      tasks[ii].elapsedTime = 0;  //reset the elapsed time
      //Serial.print("ISR");
      //Serial.print(ii);
    }
  }
}


void zeroing(void)
{
  bool lims[3] = {false};
  int motor_speed = -5, i;

  while (!lims[0])
  {
    tasks[0].period = myStepper[0].newFrequency(myEncoder[0].read(), myEncoder[0].read() + motor_speed );
    for (i = 0; i < NUM_JOINTS; ++i) {        if(i != 0)    {    tasks[i].period = myStepper[i].newFrequency(myEncoder[i].read(), 0 );     } } /// Keep other motors where they are
    Serial.print("Check 1: ");
    print_lim_swithces( lims[0] , lims[1], lims[2] );
    delay(5);
    lims[0] = readGPIOFast(lim_switch_a);
  }

  initEncoders();


  while (!lims[1])
  {
    tasks[1].period = myStepper[1].newFrequency(myEncoder[1].read(), myEncoder[1].read() + motor_speed );
    for (i = 0; i < NUM_JOINTS; ++i) {        if(i != 1)    {    tasks[i].period = myStepper[i].newFrequency( myEncoder[i].read(), 0 );     } } /// Keep other motors where they are

    Serial.print("Check 2: ");
    print_lim_swithces( lims[0] , lims[1], lims[2] );
    delay(5);
    lims[1] = readGPIOFast(lim_switch_b);
  }

  initEncoders();


  while (!lims[2])
  {
    tasks[2].period = myStepper[2].newFrequency( myEncoder[2].read(), myEncoder[2].read() + motor_speed );
    for (i = 0; i < NUM_JOINTS; ++i) {        if(i != 2)    {    tasks[i].period = myStepper[i].newFrequency( myEncoder[i].read(), 0 );     } } /// Keep other motors where they are
    Serial.print("Check 3: ");
    print_lim_swithces( lims[0] , lims[1], lims[2] );
    delay(5);
    lims[2] = readGPIOFast(lim_switch_c);
  }

  initEncoders();

  for (i = 1; i < NUM_JOINTS; ++i) {    tasks[i].period = myStepper[i].newFrequency( myEncoder[i].read(), 0 );     }  /// Keep all motors where they are

}


bool readGPIOFast(int pin) {
  return *(portInputRegister(pin)) & digitalPinToBitMask(pin);
}




//****************************************    Print Functions       ****************************************


void print_lim_swithces (bool a, bool b , bool c)
{
  Serial.print(a);
  Serial.print(b);
  Serial.print(c);
  Serial.println(" ");
}



void print_econder_values (void)
{
  // Encodor readout  
  for ( int i = 0; i < NUM_JOINTS; ++i)
  {
    Serial.print(myEncoder[i].read());
    Serial.print(" ");
  }
  Serial.println(" ");
}


void print_target_values (void)
{
  // Encodor readout  
  for ( int i = 0; i < NUM_JOINTS; ++i)
  {
    Serial.print(position_cmds[i]);
    Serial.print(" ");
  }
  Serial.println(" ");
}

/*

If the permision port permision error accures, use the following commands on Jetson

sudo usermod -a -G tty <username>
sudo usermod -a -G dialout <username>
sudo usermod -a -G sudo <username>

sudo chmod g+rw /dev/ttyTHS1


*/