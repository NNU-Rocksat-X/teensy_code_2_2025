/**
 * TODO: write a file description
 */
#ifndef step_h
#define step_h

//#include "Arduino.h"

class Stepper{
  public:
    Stepper(int _step_pin, int _dir_pin, int encoderResolution, int _motor_id, bool _closed_loop);    // Deleted pramaters: int _enc_pin_A, int enc_pin_B,
    void step();
    int newFrequency(double position, double desired_position);
    double current_velocity;
    double velocity;
    int motor_id;
    bool closed_loop;

  private:
    int step_pin;
    int dir_pin;
    bool direction;
    int motorFrequency;
    bool highLow;
    int encoder_resolution;

    int deg_to_step(int deg);
    double pid_controller(double desired_angle, double current_angle);

    double current_angle;
    double derivative;
    double integral;
    double error;
    double proportional_gain;
    double integral_gain;
    double derivative_gain;
    double max_integral;

    double previous_error;
    int previous_time;
    
};

#endif
