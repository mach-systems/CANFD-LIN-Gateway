/*
 * lin_timer.h
 *
 *  Created on: Sep 18, 2020
 *      Author: Karel Hevessy
 */

#ifndef INC_LIN_TIMER_H_
#define INC_LIN_TIMER_H_

#define LIN_TIMEOUT_TIMER     TIM7

/*
 * Initialize timer 7 to generate interrupt every Tbit (1 / baudrate).
 */
void LinTimeoutTimerInit(void);

/*
 *  Start Tbit timer. Also resets its value.
 */
void LinTimeoutTimerStart(void);

/*
 * Stop Tbit timer.
 */
void LinTimeoutTimerStop(void);

/*
 * One timer period elapsed.
 */
void LinTimeoutTimerIrqCallback(void);



#endif /* INC_LIN_TIMER_H_ */
