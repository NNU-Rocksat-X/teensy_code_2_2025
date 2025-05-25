/**
 * Stepper class for PID control of stepper motors. 
 * 
 * @author Riley Mark
 * @author December 12, 2022
 */
#ifndef step_h
#define step_h

#define EJECTOR_MOTOR_SPEED 100

#include "Arduino.h"

class Stepper{
  public:
    double current_velocity;
    double velocity;
    double current_angle;
    int motor_id;
    bool closed_loop;

    /**
     * Stepper class constructor 
     */
    Stepper(int _step_pin, int _dir_pin, int encoderResolution, int _motor_id, bool _closed_loop);    // Deleted pramaters: int _enc_pin_A, int enc_pin_B,
    
    /**
     * Advances the state of the step pin 
     * 
     * @return - none
     */
    void step(void);

    /**
     * Function to update the frequency and direction of the motor movement.
     * 
     * @param position - current encoder position
     * @param desired_position - desired position from Jetson cmd
     * 
     * @return int - frequency of motor movement
     */
    int newFrequency(double position, double desired_position);

  private:
    int step_pin;
    int dir_pin;
    bool direction;
    int motorFrequency;
    bool highLow;
    int encoder_resolution;

    double derivative;
    double integral;
    double error;
    double proportional_gain;
    double integral_gain;
    double derivative_gain;
    double max_integral;

    double previous_error;
    int previous_time;

    /**
     * Converts degrees to steps
     * 
     * @param deg - int of target position in degrees
     * TODO: For future missions this should get converted to a double
     * 
     * @return int - angle in steps
     */
    int deg_to_step(int deg);

    /**
     * The PID controller for the ARM motors.
     * 
     * @param desired_angle - position setpoint in encoder steps
     * @param current_angle - current position of the motor in encoder steps
     * 
     * @return int - current instantaneous velocity of motor
     */    
    double pid_controller(double desired_angle, double current_angle);

};

#endif
