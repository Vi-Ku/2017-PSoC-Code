/* ========================================
 * BYU Mars Rover 2016
 * Authors: Marshall Garey, Rodolfo Alberto
 * ========================================
*/
#include "pololuControl.h"

void pololuControl_turnMotorOff(uint8_t joint) {
    switch(joint) {
    case POLOLUCONTROL_TURRET:
        UART_Turret_WriteTxData(POLOLUCONTROL_MOTOR_OFF_COMMAND);
        break;
    case POLOLUCONTROL_SHOULDER:
        UART_Shoulder_WriteTxData(POLOLUCONTROL_MOTOR_OFF_COMMAND);
        break;
    case POLOLUCONTROL_ELBOW:
        UART_Elbow_WriteTxData(POLOLUCONTROL_MOTOR_OFF_COMMAND);
        break;
    case POLOLUCONTROL_FOREARM:
        UART_Forearm_WriteTxData(POLOLUCONTROL_MOTOR_OFF_COMMAND);
        break;
    default:
        break;
    }
}

void pololuControl_driveMotor(uint16_t target, uint8_t joint) {
    if (target > POLOLUCONTROL_MAX_TARGET)
        target = POLOLUCONTROL_MAX_TARGET;
    else if (target < POLOLUCONTROL_MIN_TARGET)
        target = POLOLUCONTROL_MIN_TARGET;
    
    //Computation for converting the target to a binary format
    // that the pololu understands
    static uint8_t serialBytes[2];
    serialBytes[0] = 0xC0 + (target & 0x1F);
    serialBytes[1] = (target >> 5) & 0x7F;
    
    switch(joint) {
    case POLOLUCONTROL_TURRET:
        UART_Turret_PutArray(serialBytes, 2);
        break;
    case POLOLUCONTROL_SHOULDER:
        UART_Shoulder_PutArray(serialBytes, 2);
        break;
    case POLOLUCONTROL_ELBOW:
        UART_Elbow_PutArray(serialBytes, 2);
        break;
    case POLOLUCONTROL_FOREARM:
        UART_Forearm_PutArray(serialBytes, 2);
        break;
    default:
        break;
    }
}

void pololuControl_readVariable(uint8_t command, uint8_t joint) {
    // first, make sure it is a valid command:
    if (command == POLOLUCONTROL_READ_FEEDBACK_COMMAND) {
        switch(joint) {
        case POLOLUCONTROL_TURRET:
            UART_Turret_WriteTxData(command);
            break;
        case POLOLUCONTROL_SHOULDER:
            UART_Shoulder_WriteTxData(command);
            break;
        case POLOLUCONTROL_ELBOW:
            UART_Elbow_WriteTxData(command);
            break;
        case POLOLUCONTROL_FOREARM:
            UART_Forearm_WriteTxData(command);
            break;
        default:
            break;
        }
    }
	else {
		// TODO: error
	}
}

/* [] END OF FILE */
