/* ========================================
 * BYU Mars Rover 2016
 * Authors: Marshall Garey, Rodolfo Alberto
 * ========================================
*/
#ifndef __ISR_HANDLER_H__ 
#define __ISR_HANDLER_H__ 

#include <project.h>

#define DEBUG_MODE 0

int compRxEventHandler();
void scienceEventHandler();
void heartbeatEventHandler();
void updateTurretPos();
void updateShoulderPos();
void updateElbowPos();
void updateForearmPos();

// Controls the h-bridges
void control_chutes(uint8_t byte);

// event variables
extern volatile uint32_t events;

#define SERVO_NEUTRAL 1500
#define SERVO_MAX 2000
#define SERVO_MIN 1000

// events
#define COMP_RX_EVENT 0x0001
#define HEARTBEAT_EVENT 0x0002

// science sensor feedback event
#define SCIENCE_EVENT 0x0004

// positional feedback events
#define TURRET_POS_EVENT 0x0010
#define SHOULDER_POS_EVENT 0x0020
#define ELBOW_POS_EVENT 0x0040
#define FOREARM_POS_EVENT 0x0080

// Hand
void driveHand(uint16_t pos);

// general macros
#define SUCCESS 0
#define UART_READ_ERROR 1
#define MESSAGE_ERROR 2

#endif
/* [] END OF FILE */
