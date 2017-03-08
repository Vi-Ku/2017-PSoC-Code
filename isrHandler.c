/* ========================================
 * BYU Mars Rover 2016
 * Authors: Marshall Garey, Rodolfo Alberto
 *
 * 2017 Revisions:
 * Taylor Welker
 * ========================================
*/
#include "isrHandler.h"
#include "pololuControl.h"
#include <stdio.h>

// Sends feedback to the on-board computer
static void feedbackToOnboardComputer();

// Displays feedback in a readable format to a terminal
// For debugging only
static void feedbackToTerminal();

// Generates fake science data and outputs to UART
static void generateScienceTestData();

// Container for all events. See isrHandler.h for macros that define the events.
volatile uint32_t events = 0;

// Arm payload struct
// These are the last positions we wrote to the motors
static struct Payload {
    uint16_t leftWheels;    //The following four are for the drive wheels
    uint8_t leftWheelsDir;
    uint16_t rightWheels;
    uint8_t rightWheelsDir;
    uint8_t chutes;         //Each bit represents a different chute
    uint16_t turretDest;    //The following four are for the arm/Science module
    uint16_t shoulderDest;
    uint16_t elbowDest;
    uint16_t forearmDest;
    uint8_t handDest;       //I am not sure what we use this for... -Taylor Welker
} Payload;
/*
//I am pretty certain there is no longer a PSoC Slave, so ignore tihs struct -Taylor Welker
static struct {
    uint8_t camSelect;
    uint8_t chuteSelect;
} PSoC_Slave_Payload;
*/
// Arm positions
// These are the most recent positions received as feedback from the motors
// We really don't need feedback from the hand since it's just open/close.
volatile uint16_t turretPos;    //Connected to the science plunge
volatile uint16_t shoulderPos;  //Connected to the science plate
volatile uint16_t elbowPos;     //Connected to the science drill
volatile uint16_t forearmPos;   //Connected to the Science Elevator

// Science sensor data
volatile int16_t temperature = 0;
volatile int16_t humidity = 0;

//WORK ON THIS///////////////////////////////////////////////////////////////////////////////////////////////////////

#define POSITION_PAYLOAD_SIZE (13) // 1 start byte, 2 bytes per joint, 6 joints, 4 bytes of science data
// The positions are stored in little endian format - low byte first, then
// high byte, in order from joints closest to the rover outward
// [turretlo, turrethi, shoulderlo, shoulderhi, elbowlo, elbowhi,
// forearmlo, forearmhi, temperaturelo, temperaturehi, humiditylo, 
// humidityhi]
static uint8_t feedbackArray[POSITION_PAYLOAD_SIZE];

// State machine states to receive commands from computer
// The state machine is defined in the function compRxEventHandler
#define PREAMBLE0 0xEA
static enum compRxStates_e { pre0, leftlo, lefthi, leftdir, rightlo, 
    righthi, rightdir, turretlo, turrethi, shoulderlo, shoulderhi, 
    elbowlo, elbowhi, forearmlo, forearmhi, handlo, chutes} compRxState;

// Receive a message from the computer
int compRxEventHandler() {
    // get next element in uart rx buffer
    static uint16_t data;
    static uint8_t byte;
    
    // Keep reading the rx buffer until empty.
    // GetRxBufferSize gets the number of bytes in the software buffer,
    // but not the hardware FIFO (which is 4 bytes in size), so we also want
    // to call ReadRxStatus and check if the RX_STS_FIFO_NOTEMPTY bit was set.
    while (UART_Computer_GetRxBufferSize() || 
          (UART_Computer_ReadRxStatus() & UART_Computer_RX_STS_FIFO_NOTEMPTY))
    {
        // MSB contains status, LSB contains data; if status is nonzero, an 
        // error has occurred
        data = UART_Computer_GetByte();
        
        // check status
        // TODO: maybe make an error LED and use this in the main function
        if (data & 0xff00) {
            return UART_READ_ERROR;
        }
        
        // mask the data to a single byte
        byte = data & 0xff;
        
        // state machine
        switch(compRxState) {
            
            // Check preamble byte
        case pre0:
            if (byte == PREAMBLE0) {
                compRxState = leftlo; // change state
            }
            break;
            
            // Actuate wheels - left and right
        case leftlo:
            Payload.leftWheels = byte;
            compRxState = lefthi; // change state
            break;
        case lefthi:
            Payload.leftWheels |= byte << 8;
            PWM_Drive_WriteCompare1(Payload.leftWheels);
            compRxState = leftdir; // change state
            break;            
        //According to the MDC151-050301 motor driver user's manual
        //Logic "1" (open) - Clockwise and Logic "0" - Counterclockwise
        case leftdir:
            //We must be sure to invert the byte for the left wheels, as they will
            //spin in the opposite direction of the right wheels when moving forward
            //or backward (right spins clockwise (1) and left spins counterclockwise (0)
            leftWheelDir_Write(!byte);
            compRxState = rightlo;
            break;       
        case rightlo:
            Payload.rightWheels = byte;
            compRxState = righthi; // change state
            break;
        case righthi:
            Payload.rightWheels |= byte << 8;
            PWM_Drive_WriteCompare2(Payload.rightWheels);
            compRxState = rightdir; // change state
            break;
        //Assuming that the pin is a simple digital output pin
        case rightdir:
            rightWheelDir_Write(byte);    //Simply grab the direction (1 or 0), set pin accordingly
            compRxState = turretlo;  //Go to the next state
            break;         
        // Actuate first 4 arm joints: turret, shoulder, elbow, forearm
        case turretlo:
            Payload.turretDest = byte;
            compRxState = turrethi; // change state
            break;
        case turrethi:
            Payload.turretDest |= byte << 8;
            pololuControl_driveMotor(Payload.turretDest,
                POLOLUCONTROL_TURRET);
            compRxState = shoulderlo; // change state
            break;
        case shoulderlo:
            Payload.shoulderDest = byte;
            compRxState = shoulderhi; // change state
            break;
        case shoulderhi:
            Payload.shoulderDest |= byte << 8;
            pololuControl_driveMotor(Payload.shoulderDest, 
                POLOLUCONTROL_SHOULDER);
            compRxState = elbowlo; // change state
            break;
        case elbowlo:
            Payload.elbowDest = byte;
            compRxState = elbowhi; // change state
            break;
        case elbowhi:
            Payload.elbowDest |= byte << 8;
            pololuControl_driveMotor(Payload.elbowDest,
                POLOLUCONTROL_ELBOW);
            compRxState = forearmlo; // change state
            break;
        case forearmlo:
            Payload.forearmDest = byte;
            compRxState = forearmhi; // change state
            break;
        case forearmhi:
            Payload.forearmDest |= byte << 8;
            pololuControl_driveMotor(Payload.forearmDest,
                POLOLUCONTROL_FOREARM);
            compRxState = handlo; // change state
            break;            
            // actuate hand
        case handlo:
            Payload.handDest = byte; // get new hand value
            driveHand(Payload.handDest); // update hand position
            compRxState = chutes; // change state
            break;            
            // actuate chutes
        case chutes:
            // byte: box open/close | chute_en | c6 | c5 | c4 | c3 | c2 | c1
            if (byte & 0x40) {
                //chute_en_Write(1);
                control_chutes(byte);
            }
            else {
                //chute_en_Write(0);
            }
            
            // box lid open/close is 8th bit
            if (byte & 0x80) {
                PWM_BoxLid_WriteCompare(SERVO_MIN); // open box
            }
            else {
                PWM_BoxLid_WriteCompare(SERVO_MAX); // close box
            }
            compRxState = pre0; // change state
            break;
        default:
            // shouldn't get here!!!
            break;
        }
    }
    
    // Check if any data came in that we didn't get. If so, then queue up
    // this event again in the events variable.
    if (UART_Computer_GetRxBufferSize() || 
        UART_Computer_ReadRxStatus() & UART_Computer_RX_STS_FIFO_NOTEMPTY) 
    {
        events |= COMP_RX_EVENT;
    }
    return SUCCESS; // success
}

void scienceEventHandler() {
    // Get feedback from science sensors: temperature and humidity
    enum states_e { pre0, pre1, templo, temphi, humlo, humhi };
    static enum states_e state = templo;
    static uint16_t temp = 0; // temporary data storage
    
    // Read until finished getting all bytes in buffer
    while (UART_ScienceMCU_GetRxBufferSize() || 
          (UART_ScienceMCU_ReadRxStatus() & UART_ScienceMCU_RX_STS_FIFO_NOTEMPTY))
    {
        // Get next byte from UART
        int16_t byte;
        byte = UART_ScienceMCU_GetByte();
        if (byte & 0xff00) {
            return; // Error - ignore byte
        }
        
        switch (state) {
        // Preamble:
        case pre0:
            if (byte == 0xff) {
                state = pre1;
            }
            break;
        // Preamble 2nd byte:
        case pre1:
            if (byte == 0x9e) {
                state = templo;
            }
            else {
                state = pre0;
            }
            break;
        // Temperature low byte
        case templo:
            temp = byte;
            state = temphi;
            break;
        // Temperature high byte
        case temphi:
            temp |= 0xff00 & (byte << 8);
            if (temp <= 100) {
                temperature = temp; // now assign temperature to this value
            }
            state = humlo;
            break;
        // Humidity low byte
        case humlo:
            temp = byte;
            state = humhi;
            break;
        // Humidity high byte
        case humhi:
            temp |= 0xff00 & (byte << 8);
            if (humidity <= 1023) {
                humidity = temp; // now assign humidity to this value
            }
            state = pre0;
            break;
        // Shouldn't ever get here
        default:
            break;
        }
    }
}

// Report current positions and ask the pololus for updated positions
void heartbeatEventHandler() {
    
    #if DEBUG_MODE
    //generateScienceTestData(); // use this to generate fake science data
    feedbackToTerminal(); // use this to see output on a terminal
    #else
    feedbackToOnboardComputer(); // use this to send to on-board computer
    #endif

    // Ask Arduino for science sensor data
    UART_ScienceMCU_PutChar(0xae); // preamble
    UART_ScienceMCU_PutChar(1); // get feedback
    
    // Get Arm feedback:
    // Turret
    pololuControl_readVariable(POLOLUCONTROL_READ_FEEDBACK_COMMAND,
		POLOLUCONTROL_TURRET);
    
    // Shoulder
    pololuControl_readVariable(POLOLUCONTROL_READ_FEEDBACK_COMMAND,
		POLOLUCONTROL_SHOULDER);
    
    // Elbow
    pololuControl_readVariable(POLOLUCONTROL_READ_FEEDBACK_COMMAND,
		POLOLUCONTROL_ELBOW);
    
    // Forearm
    pololuControl_readVariable(POLOLUCONTROL_READ_FEEDBACK_COMMAND,
		POLOLUCONTROL_FOREARM);
}

// ===========================================================================
// Helper and debug function definitions
// ===========================================================================

// Control hand
void driveHand(uint16_t pos) {
    if (pos == 1) { // open (retract linear actuators)
        hand_a_Write(1);
        hand_b_Write(0);
        hand_en_Write(1); //Enable the hand if you want it to move
    }
    else if (pos == 2) { // close (extend linear actuators)
        hand_b_Write(1);
        hand_a_Write(0);
        hand_en_Write(1); //Enable the hand if you want it to move
    }
    else { // don't move
        hand_a_Write(0);
        hand_b_Write(0);
        hand_en_Write(0); //Don't enable the hand if you don't want it to move
    }
}

// Update turret position
void updateTurretPos() {
    static enum states_e {low, high} state = low;
    
    static uint16_t temp = 0;
    while(UART_Turret_ReadRxStatus() & UART_Turret_RX_STS_FIFO_NOTEMPTY) {
        switch(state) {
            case low:
                temp = UART_Turret_GetByte() & 0xff;
                state = high;
            break;
            case high:
                temp |= (UART_Turret_GetByte() << 8) & 0xff00;
                if (temp <= 4095) {
                    turretPos = temp;
                }
                state = low;
            break;
            default:
                state = low;
            break;
        }
    }
}

// Update shoulder position
void updateShoulderPos() {
    static enum states_e {low, high} state = low;
    
    static uint16_t temp;
    while(UART_Shoulder_ReadRxStatus() & UART_Shoulder_RX_STS_FIFO_NOTEMPTY) {
        switch(state) {
            case low:
                temp = UART_Shoulder_GetByte() & 0xff;
                state = high;
            break;
            case high:
                if (temp <= 4095) {
                    temp |= (UART_Shoulder_GetByte() << 8) & 0xff00;
                }
                shoulderPos = temp;
                state = low;
            break;
            default:
                state = low;
            break;
        }
    }
}

// Update elbow position
void updateElbowPos() {
	static enum states_e {low, high} state = low;
    
    static uint16_t temp;
    while(UART_Elbow_ReadRxStatus() & UART_Elbow_RX_STS_FIFO_NOTEMPTY) {
        switch(state) {
            case low:
                temp = UART_Elbow_GetByte() & 0xff;
                state = high;
            break;
            case high:
                if (temp <= 4095) {
                    temp |= (UART_Elbow_GetByte() << 8) & 0xff00;
                }
                elbowPos = temp;
                state = low;
            break;
            default:
                state = low;
            break;
        }
    }
}

// Update forearm position
void updateForearmPos() {
	static enum states_e {low, high} state = low;
    
    static uint16_t temp;
    while(UART_Forearm_ReadRxStatus() & UART_Forearm_RX_STS_FIFO_NOTEMPTY) {
        switch(state) {
            case low:
                temp = UART_Forearm_GetByte() & 0xff;
                state = high;
            break;
            case high:
                if (temp <= 4095) {
                    temp |= (UART_Forearm_GetByte() << 8) & 0xff00;
                }
                forearmPos = temp;
                state = low;
            break;
            default:
                state = low;
            break;
        }
    }
}

void control_chutes(uint8_t byte) {
    // chutes 1-6 are bits 0-5;
    
    // chute 1
    if (byte & 0x1) // close
    {
        chute1b_Write(0);
        chute1a_Write(1);
    }
    else // open
    {
        chute1a_Write(0);
        chute1b_Write(1);
    }
    // chute 2
    if (byte & 0x2) // close
    {
        chute2b_Write(0);
        chute2a_Write(1);
    }
    else // open
    {
        chute2a_Write(0);
        chute2b_Write(1);
    }
    // chute 3
    if (byte & 0x4) // close
    {
        chute3b_Write(0);
        chute3a_Write(1);
    }
    else // open
    {
        chute3a_Write(0);
        chute3b_Write(1);
    }
    // chute 4
    if (byte & 0x8) // close
    {
        chute4b_Write(0);
        chute4a_Write(1);
    }
    else // open
    {
        chute4a_Write(0);
        chute4b_Write(1);
    }
}

// Send feedback to computer
static void feedbackToOnboardComputer() {
    feedbackArray[0] = 0xE3; // start byte;
    feedbackArray[1] = (turretPos & 0xff);
    feedbackArray[2] = ((turretPos >> 8) & 0xff);
    feedbackArray[3] = (shoulderPos & 0xff);
    feedbackArray[4] = ((shoulderPos >> 8) & 0xff);
    feedbackArray[5] = (elbowPos & 0xff);
    feedbackArray[6] = ((elbowPos  >> 8) & 0xff);
    feedbackArray[7] = (forearmPos & 0xff);
    feedbackArray[8] = ((forearmPos >> 8) & 0xff);
	feedbackArray[9] = ((temperature & 0xff));
	feedbackArray[10] = ((temperature >> 8) & 0xff);
	feedbackArray[11] =((humidity & 0xff));
	feedbackArray[12] =((humidity >> 8) & 0xff);
	UART_Computer_PutArray(feedbackArray, POSITION_PAYLOAD_SIZE);
}

// A debugging function to see output on a terminal
static void feedbackToTerminal() {
    //static int i = 0;
    //i++;
    //turretPos += i;
    //shoulderPos += 2*i;
    //elbowPos += 3*i;
    //forearmPos += 4*i;
    //temperature = 5*i;
    //humidity = 6*i;
    
    char pos[34];
    sprintf(pos, "\n\r\n\rpositions:%4d,%4d,%4d,%4d", 
        turretPos, shoulderPos, elbowPos, forearmPos);
    pos[33] = 0; // null terminate
    char tem[20];
    sprintf(tem, "%d", temperature);
    tem[19] = 0; // null terminate
    char hum[20];
    sprintf(hum, "%d", humidity);
    hum[19] = 0; // null terminate
    UART_Computer_PutString(pos);
    UART_Computer_PutString("\n\rtemp:");
    UART_Computer_PutString(tem);
    UART_Computer_PutString("\n\rhumid:");
    UART_Computer_PutString(hum);
}

// Sends pretend data out on science uart
static void generateScienceTestData() {
    static uint16_t hum = 0;
    static uint16_t temp = 0;
    hum++;
    temp--;
    static uint8_t array[6];
    array[0] = 0xff;
    array[1] = 0xe9;
    array[2] = (uint8_t)(temp & 0xff);
    array[3] = (uint8_t)(temp >> 8) & 0xff;
    array[4] = (uint8_t)(hum & 0xff);
    array[5] = (uint8_t)(hum >> 8) & 0xff;
    UART_ScienceMCU_PutArray(array, 6);
}

/* [] END OF FILE */
