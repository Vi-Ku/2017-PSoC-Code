/* ========================================
 * BYU Mars Rover 2016
 * Authors: Marshall Garey, Rodolfo Alberto
 * ========================================
*/
#ifndef __ISR_H__
#define __ISR_H__

#include <project.h>

CY_ISR_PROTO(CompRxISR);
CY_ISR_PROTO(HeartbeatISR);
CY_ISR_PROTO(TurretRxISR);
CY_ISR_PROTO(ShoulderRxISR);
CY_ISR_PROTO(ElbowRxISR);
CY_ISR_PROTO(ForearmRxISR);
CY_ISR_PROTO(ScienceRxISR);

#endif

/* [] END OF FILE */
