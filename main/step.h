#ifndef step_h
#define step_h

//#include "Arduino.h"

class Stepper{
  public:
    Stepper(int _step_pin, int _dir_pin, int encoderResolution, int _motor_id);    // Deleted pramaters: int _enc_pin_A, int enc_pin_B,
    void step();
    int newFrequency(double position, double desired_position);
    int readEncoders();
    void currentVelocity(int currentPosition);  // populates the current velocity values on all the instances of this class
    double current_velocity;
    double velocity;

  private:
    int step_pin;
    int dir_pin;
    bool direction;
    int motorFrequency;
    bool highLow;
    int encoder_resolution;
    int motor_id;

    int deg_to_step(int deg);
    double pid_controller(double desired_angle, double current_angle);

    double zero_position;
    double current_angle;
    double derivative;
    double integral;
    double error;
    double proportional_gain;
    double integral_gain;
    double derivative_gain;
    double previous_error;
    int previous_time;
    //double velocity;
    
};

#endif
