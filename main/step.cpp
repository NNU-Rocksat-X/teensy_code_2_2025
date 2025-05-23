#include "step.h"
#include <Arduino.h>

Stepper::Stepper(int _step_pin, int _dir_pin, int encoderResolution, int _motor_id) { // Deleted pramaters: int _enc_pin_A, int enc_pin_B,

  step_pin = _step_pin;
  dir_pin = _dir_pin;
  encoder_resolution = encoderResolution;
  motor_id = _motor_id;

  pinMode(step_pin, OUTPUT);
  pinMode(dir_pin, OUTPUT);
}

void Stepper::step() {
  if (!highLow) {
    if (direction == HIGH) {
      digitalWrite(dir_pin, HIGH);
      digitalWrite(step_pin, HIGH);
    }

    else {
      digitalWrite(dir_pin, LOW);
      digitalWrite(step_pin, HIGH);
    }

    highLow = HIGH;
  }

  //delaymicroseconds(10)

  else {
    digitalWrite(step_pin, LOW);
    highLow = LOW;
  }
}


double Stepper::pid_controller(double desired_angle, double current_angle) {
  double proportional_gain = 0.3;
  double integral_gain = 0.5;
  double derivative_gain = 0.01;
  double max_integral = 1;
  //double setPoint = 200;

  double now_time = micros();
  double delta_time = now_time - previous_time;
  previous_time = now_time;

  error = desired_angle - current_angle;
  integral += error;  //* delta_time;
  derivative = (error - previous_error) / delta_time;

  if (integral > max_integral) {  // stop the integral from getting out of control
    integral = max_integral;
  } else if (integral < -max_integral) {
    integral = -max_integral;
  }
  /*
  if (abs(error) < 5) {              // reset integral if the error is enough
    integral = 0;
    //derivative = 0;
    //error = 0;
  }*/

  //if (error > set_point) {
  //  velocity += 15;
  //}

  previous_error = error;

  return velocity = error * proportional_gain + integral * integral_gain + derivative * derivative_gain;
}

int Stepper::deg_to_step(int deg) {
  if (motor_id == 2 || motor_id == 4) {
    int temp_val = ((encoder_resolution * 4.0 * 6.0 * 6.0) / 360.0);
    return temp_val * deg;
  } else if (motor_id == 1) {
    int temp_val = ((encoder_resolution * 4.0 * 3.0) / 360.0);
    return temp_val * deg;
  } else {
    int temp_val = ((encoder_resolution * 4.0 * 6.0) / 360.0);
    return temp_val * deg;
  }
}



int Stepper::newFrequency(double position, double desired_position) {
  int velocity;

  velocity = pid_controller(desired_position, position);

  if ( motor_id == 2 || motor_id == 3 || motor_id == 4 || motor_id == 6) {  // this is choosing the direction of the motors. If the motor spins out of control its is likely that the encoder was wired backwards and can easily be fixed by adding or removing the motor to this if statement
    if (velocity > 0) {
      direction = HIGH;
    } else {
      direction = LOW;
    }
  } else {    
    if (velocity > 0) {
      direction = LOW;
    } else {
      direction = HIGH;
    }
  }

  if (fabs(velocity) < 0.001) {
    motorFrequency = 1000000;
    //digitalWrite(13, LOW);
  } else {
    motorFrequency = (1000 / abs(velocity));  // this just needs to invert the velocity values
    //digitalWrite(13, HIGH);
  }

  if (motorFrequency <= 100) {
    motorFrequency = 100;
  } else {
  }

  //take steps/s convert to s/step and convert to microseconds

  return motorFrequency;
}

/*
void Stepper::currentVelocity(int currentPosition){
  int counts_per_step = (encoder_resolution * 1.8)/ (360.0); // counts per step with a resolution of 

  current_vel_time = millis();

  delta_time = current_vel_time - previous_vel_time;

  current_velocity = ((1 / counts_per_step) * (currentPosition - previous_vel_position)) / delta_time; // steps per microsecond 

  previous_vel_time = current_vel_time;
  previous_vel_position = currentPosition;

  if((1 / current_velocity) < motorFrequency) {
    digitalWrite(13, HIGH);
  }
  else {
    digitalWrite(13, LOW);
  }
}*/


/*
Encoder encoder(enc_pin_A, enc_pin_B)
*/

/*
int readEncoders() {
  return encoder.read();
}
*/
