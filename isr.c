/* ========================================
 * BYU Mars Rover 2016
 * Authors: Marshall Garey, Rodolfo Alberto
 * ========================================
*/
#include "isr.h"
#include "isrHandler.h"

//CY_ISR_PROTO(CompRxISR);
CY_ISR(CompRxISR) {
    // clear interrupt on uart module
    UART_Computer_GetRxInterruptSource();

    // Clear the pending interrupt
    //NOTE: the interrupt for the UART will stay high so long as 
    //the FIFO buffer has data in it. 
    Comp_RX_ISR_ClearPending();
    
    // queue up computer rx event
    events |= COMP_RX_EVENT;
}

//CY_ISR_PROTO(TurretRxISR);
CY_ISR(TurretRxISR) {
    // Clear interrupt
    UART_Turret_GetRxInterruptSource();
    TurretRxIsr_ClearPending();
    
    // Queue event
    events |= TURRET_POS_EVENT;
}

//CY_ISR_PROTO(ShoulderRxISR);
CY_ISR(ShoulderRxISR) {
    // Clear interrupt
    UART_Shoulder_GetRxInterruptSource();
    ShoulderRxIsr_ClearPending();
    
    // Queue event
    events |= SHOULDER_POS_EVENT;
}

//CY_ISR_PROTO(ElbowRxISR);
CY_ISR(ElbowRxISR) {    
    // clear interrupt
    UART_Elbow_GetRxInterruptSource();
    ElbowRxIsr_ClearPending();
    
    // queue event
    events |= ELBOW_POS_EVENT;
}

//CY_ISR_PROTO(ForearmRxISR);
CY_ISR(ForearmRxISR) {
    // clear interrupt
    UART_Forearm_GetRxInterruptSource();
    ForearmRxIsr_ClearPending();
    
    // queue event
    events |= FOREARM_POS_EVENT;
}

//CY_ISR_PROTO(PlateShiftRxISR);
CY_ISR(PlateShiftRxISR) {
    // clear interrupt
    UART_PlateShift_GetRxInterruptSource();
    PlateShiftRxIsr_ClearPending();
    
    // queue event
    events |= PLATESHIFT_POS_EVENT;
}

//CY_ISR_PROTO(ScienceRxISR);
CY_ISR(ScienceRxISR) {
    UART_ScienceMCU_ReadRxStatus(); // clear interrupt
    ScienceRxIsr_ClearPending();
    
    events |= SCIENCE_EVENT;
}

//CY_ISR_PROTO(HeartbeatISR);
CY_ISR(HeartbeatISR) {
    static unsigned count = 0;
    
    // clears interrupt on counter module
    PWM_Drive_ReadStatusRegister();
    heartbeatIsr_ClearPending();
    
    // Use count as a divider - only queue up event every 5 interrupts
    count++;
    if (count >= 5) {
        count = 0;
        // queue up heartbeat event
        events |= HEARTBEAT_EVENT;
    }
}

/* [] END OF FILE */
