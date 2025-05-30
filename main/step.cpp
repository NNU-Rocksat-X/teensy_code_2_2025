/**
 * Stepper class for PID control of stepper motors. 
 * 
 * @author Riley Mark
 * @author December 12, 2022
 */
#include "step.h"
// #include <Arduino.h>

/**
 * Stepper class constructor 
 */
Stepper::Stepper (int _step_pin, 
                  int _dir_pin,  
                  int encoderResolution, 
                  int _motor_id,
                  bool _closed_loop) 
{
  step_pin = _step_pin;
  dir_pin = _dir_pin;
  encoder_resolution = encoderResolution;
  motor_id = _motor_id;
  closed_loop = _closed_loop;

  proportional_gain = 0.3;
  integral_gain = 0.5;
  derivative_gain = 0.01;
  max_integral = 1;

  pinMode(step_pin, OUTPUT);
  pinMode(dir_pin, OUTPUT);
}


/**
 * Advances the state of the step pin 
 * 
 * @return - none
 */
void Stepper::step () 
{
  if (!highLow) 
  {
    if (direction == HIGH) 
    {
      digitalWrite(dir_pin, HIGH);
      digitalWrite(step_pin, HIGH);
    }
    else 
    {
      digitalWrite(dir_pin, LOW);
      digitalWrite(step_pin, HIGH);
    }

    highLow = HIGH;
  }
  else 
  {
    digitalWrite(step_pin, LOW);
    highLow = LOW;
  }
}


/**
 * The PID controller for the ARM motors.
 * 
 * @param desired_angle - position setpoint in encoder steps
 * @param current_pos - current position of the motor in encoder steps
 * 
 * @return int - current instantaneous velocity of motor
 * 
 * TODO: Tune the PID controllers
 */
double Stepper::pid_controller(double desired_angle, double current_pos) 
{
  double now_time = micros();
  double delta_time = now_time - previous_time;
  previous_time = now_time;

  error = desired_angle - current_pos;
  integral += error;
  derivative = (error - previous_error) / delta_time;

  // clamp the integral
  if (integral > max_integral) 
  {  
    integral = max_integral;
  } 
  else if (integral < -max_integral) 
  {
    integral = -max_integral;
  }

  previous_error = error;

  return velocity = error * proportional_gain + integral * integral_gain + derivative * derivative_gain;
}


/**
 * Converts degrees to steps
 * 
 * @param deg - int of target position in degrees
 * TODO: For future missions this should get converted to a double
 * 
 * @return int - angle in steps
 * 
 * TODO: update this with the updated gear ratios per joint
 * TODO: dont hard code in the gear ratios....
 */
int Stepper::deg_to_step(int deg) 
{
  if (motor_id == 7 || motor_id == 8 ) 
  {
    int temp_val = ( 1 / 360.0 );
    return temp_val * deg;
  } 
  else if (motor_id == 6 ) 
  {
    int temp_val = ((encoder_resolution * 4.0 ) / 360.0);
    return temp_val * deg;
  }
  else
  {
    int temp_val = ((encoder_resolution * 4.0 * 7.0 ) / 360.0);
    return temp_val * deg;
  } 
}


/**
 * Function to update the frequency and direction of the motor movement.
 * 
 * @param position - current encoder position
 * @param desired_position - desired position from Jetson cmd
 * 
 * @return int - frequency of motor movement
 * 
 * TODO: set the motor polarity once the new motors are wired up
 */
int Stepper::newFrequency(double position, double desired_position) 
{
  int velocity;

  velocity = pid_controller(desired_position, position);

  // TODO: Test the closed loop mode control of the final two joints before 
  //       modifying this further.
  // if (closed_loop)
  // {
  //   velocity = pid_controller(desired_position, position);
  // }
  // else
  // {
  //   velocity = pid_controller();
  // }

  /** TODO: Modify these based on motor polarity */
  if ( motor_id == 2 || motor_id == 3 || motor_id == 4 || motor_id == 6) 
  {  
    if (velocity > 0) 
    {
      direction = HIGH;
    } 
    else 
    {
      direction = LOW;
    }
  } 
  else 
  {    
    if (velocity > 0) 
    {
      direction = LOW;
    } 
    else 
    {
      direction = HIGH;
    }
  }

  // clamp the motor frequency
  if (fabs(velocity) < 0.001) 
  {
    motorFrequency = 1000000;
  } 
  else 
  {
    motorFrequency = (1000 / abs(velocity));
  }

  if (motorFrequency <= 100) 
  {
    motorFrequency = 100;
  } 

  return motorFrequency;
}
