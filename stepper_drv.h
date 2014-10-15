/* ----------------------------------------------------------------------------
 * File    : stepper_drv.h
 * Author  : Denis Jullien
 * Version : V2.0
 * Date    : 08/10/2014
 *
 *  Header for stepper_drv.c module
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STEPPER_DRV_H
#define STEPPER_DRV_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_tim.h"
#include "main.h"

/* Exported constants --------------------------------------------------------*/

#define EIGHTSTEPS
//#define HALFSTEPS
//#define FULLSTEPS

#define SMAX_ACCEL 100
#define SMAX_DECEL 100

// Timer 3 running on 1MHZ
#define T_FREQ 1000000.0f

// Number of (full)steps per round on stepper motor in use.
#define FSPR 200.0f

// Minimal speed before to not decelerate in speed mode
#define MIN_SPEED 10

//----------------------------------------------------------------------------

#ifdef EIGHTSTEPS
#define SPR (FSPR*8)
#endif
#ifdef HALFSTEPS
#define SPR (FSPR*2)
//#pragma message("[stepper_drv.c] *** Using Halfsteps ***")
#endif
#ifdef FULLSTEPS
#define SPR FSPR
//#pragma message("[stepper_drv.c] *** Using Fullsteps ***")
#endif
#ifndef HALFSTEPS
#ifndef FULLSTEPS
#ifndef EIGHTSTEPS
#error FULLSTEPS/HALFSTEPS not defined!
#endif
#endif
#endif

// Maths constants. To simplify maths when calculating in speed_cntr_Move().
#define PI 3.14159265f
#define ALPHA (PI*2/SPR)                    // 2*pi/spr (motor step angle (radian))
#define A_T (ALPHA*T_FREQ)
#define T_FREQ_148 (float)(T_FREQ*0.676f)
#define A_x2 (ALPHA*2.0f)

/* Exported types ------------------------------------------------------------*/

typedef enum
{
	BACKWARD = 0,
	FORWARD = 1
}MVT_DIR_t;

typedef enum
{
	STEP = 0,
	SPEED = 1
}MVT_TYPE_t;


typedef enum
{
	STOP = 0,
	ACCEL = 1,
	DECEL = 2,
	RUN   = 3
}SLOPE_STATE_t;

typedef struct {
	float max_speed;
	float accel;
	float decel;
}MOTOR_PARAM_t;

/*! \brief Holding data used by timer interrupt for speed ramp calculation.
 *
 *  Contains data used by timer interrupt to calculate speed profile.
 *  Data is written to it by move(), when stepper motor is moving (timer
 *  interrupt running) data is read/updated when calculating a new step_delay
 */
typedef struct tag_speedRampData{

	//! What part of the speed ramp we are in.
	SLOPE_STATE_t run_state;
	//! What running mode we are using : Step or Speed.
	MVT_TYPE_t run_mode;
	//! Period of next timer delay. At start this value set the acceleration rate.
	unsigned int step_delay;
	//! What step to stop accel
	unsigned int accel_stop;
	//! What step_pos to start deceleration
	unsigned int decel_start;
	//! Sets deceleration rate.
	signed int decel_val;
	//! Minimum time delay (max speed)
	unsigned int min_delay;
	//! Counter used when accelerating/decelerating to calculate step_delay.
	signed int accel_count;

	//--- for interrupt ---
	//! Remember the last step delay used when accelerating.
	int last_accel_delay;
	//! Counting steps when moving.
	unsigned int step_count;
	//! Keep track of remainder from new_step-delay calculation for accuracy
	int rest;

	//! Timer channel
	unsigned int channel;

	//! If delay is required
	unsigned char do_delay : 1;
	//! If motor is running
	volatile unsigned char running : 1;
} speedRampData;

typedef struct tag_machineAxis{

	float x;
	float y;
	float z;
	float e;

} machineAxis;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void speed_cntr_Step(speedRampData *srdX, int step, float accel, float decel, float speed);
void speed_cntr_Speed(speedRampData *srdX, float accel, float decel, float speed);
void speed_cntr_Goto(float Cx, float Cy, float Cz, float Ce,float speed);
unsigned int speed_cntr_STOP(void);
void speed_cntr_Init(void);
unsigned int speed_cntr_interrupt(speedRampData *srdX );
void setDir(DOUT_NAME_t p, MVT_DIR_t dir);


#endif
