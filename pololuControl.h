/* ========================================
 * BYU Mars Rover 2016
 * Authors: Marshall Garey, Rodolfo Alberto
 * ========================================
*/

/*These #defines are the variables that can be read
  using the readVariable function, simply pass one of
  these to the function's paramater*/
#if !defined(pololuControl_H)
#define pololuControl_H
    
#include <project.h>

//Command to turn motor off
#define POLOLUCONTROL_MOTOR_OFF_COMMAND 0xFF

//Value the pololu will try to drive the motor at
#define POLOLUCONTROL_READ_TARGET_COMMAND 0xA3

//Measurement of voltage on the FB pin
//Ranges from 0 to 4092 (5 volts)
#define POLOLUCONTROL_READ_FEEDBACK_COMMAND 0xA7      

//This is the duty cycle the pololu is trying to achieve
//Ranges from -600 (reverse) to 600 (forward)
// UNUSED!
#define POLOLUCONTROL_READ_DUTYCYCLETARGET_COMMAND 0xAB

//The value the pololu is currently driving the motor at
// UNUSED!
#define POLOLUCONTROL_READ_DUTYCYCLE_COMMAND 0xAD

//Resets to 0 after 65535
#define POLOLUCONTROL_READ_PID_PERIOD_COUNT_COMMAND 0xB1

#define POLOLUCONTROL_MAX_TARGET 4092
#define POLOLUCONTROL_MID_TARGET 2048
#define POLOLUCONTROL_MIN_TARGET 0

// macros that define which component to use:
#define POLOLUCONTROL_TURRET 0
#define POLOLUCONTROL_SHOULDER 1
#define POLOLUCONTROL_ELBOW 2
#define POLOLUCONTROL_FOREARM 3
#define POLOLUCONTROL_PLATESHIFT 4

// turn the motor off
void pololuControl_turnMotorOff(uint8_t joint);

//Grabs the current target and transmits that value to the pololu
// target - the target position
// joint - which arm joint to move, defined above
void pololuControl_driveMotor(uint16_t target, uint8_t joint);

//Commands the pololu to read the variable specified by command
// command - which variable to read, as defined above
void pololuControl_readVariable(uint8_t command, uint8_t joint);

#endif
/* [] END OF FILE */
