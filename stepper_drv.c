/* ----------------------------------------------------------------------------
 * File    : stepper_drv.c
 * Author  : Denis Jullien
 * Version : V2.0
 * Date    : 08/10/2014
 *
 *  Stepper driver with acceleration control for STM32F4
 *  From AVR446 Application note - Linear speed control of stepper motor
 * 	& http://www.embedded.com/design/embedded/4006438/Generate-stepper-motor-speed-profiles-in-real-time
 *
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * Denis Jullien wrote this file. As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return.
 * ----------------------------------------------------------------------------
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "stepper_drv.h"
#include "misc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include <math.h>
#include <stdbool.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

MOTOR_PARAM_t mPara[] = {
// Speed | Accel | Decel
	{40,	100,	100}, //X
	{40,	100,	100}, //Y
	{40,	100,	100}, //Z
	{40,	100,	100}  //E
};

//! Contains data for timer interrupt.
speedRampData stGen;
machineAxis mPos;
machineAxis sFactor;
machineAxis sCount;

TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
uint16_t PrescalerValue = 0;

uint16_t capture = 0;
uint16_t ttemp1 = 0;
uint16_t ttemp2 = 0;

float sync_val;
extern __IO float x;
extern __IO float y;
/* Private function prototypes -----------------------------------------------*/
static void pulseStepH();
static void pulseStepD();
static void calcNStep( speedRampData *srdX );
bool AlmostEqual(float A, float B);
float min4Speed(MOTOR_PARAM_t * mots, machineAxis f);
float min4Accel(MOTOR_PARAM_t * mots, machineAxis f);
float min4Decel(MOTOR_PARAM_t * mots, machineAxis f);

/******************************************************************************/
/*         		     Stepper motor step generation 	                          */
/******************************************************************************/

/*! @brief Move the stepper motor a given number of steps.
 *
 *  Makes the stepper motor move the given number of steps.
 *  It accelrate with given accelration up to maximum speed and decelerate
 *  with given deceleration so it stops at the given step.
 *  If accel/decel is to small and steps to move is to few, speed might not
 *  reach the max speed limit before deceleration starts.
 *
 *  @param step  Number of steps to move (pos - CW, neg - CCW).
 *
 *  @param accel  Accelration to use, in rad/sec^2.
 *  @param decel  Decelration to use, in rad/sec^2.
 *  @param speed  Max speed, in rad/sec.
 */
void speed_cntr_Step(speedRampData *srdX, int step, float accel, float decel, float speed)

{
	//! Number of steps before we hit max speed.
	unsigned int max_s_lim;

	// If moving only 1 step.
	if(step == 1){
		// Move one step...
		srdX->accel_count = -1;
		// ...in DECEL state.
		srdX->run_state = DECEL;
		// Just a short delay so main() can act on 'running'.
		srdX->step_delay = 1000;
		srdX->running = 1;
		//OCR1A = 10;
		// Run Timer/Counter 1 with prescaler = 8.
		TIM_ITConfig(TIM3, srdX->channel , ENABLE);
	}
	// Only move if number of steps to move is not zero.
	else if(step != 0){

		speed = fabsf(speed);
		accel = fabsf(accel);
		decel = fabsf(decel);

		// Set max speed limit, by calc min_delay to use in timer.
		// min_delay = (alpha * f) / w
		srdX->min_delay = (unsigned int) (A_T / speed);

		// Set accelration by calc the first (c0) step delay .
		// step_delay = 0.676 * f * sqrt(2*alpha/accel)
		srdX->step_delay = (unsigned int) (T_FREQ_148 * sqrtf((float) A_x2 / accel));

		// Find out after how many steps does the speed hit the max speed limit.
		// max_s_lim = speed^2 / (2*alpha*accel)
		max_s_lim = (unsigned int) (powf(speed,2.0f) / ((float) A_x2 * accel));
		// If we hit max speed limit before 0,5 step it will round to 0.
		// But in practice we need to move at least 1 step to get any speed at all.
		if(max_s_lim == 0){
			max_s_lim = 1;
		}

		// Find out after how many steps we must start deceleration (if accel does not hit max speed).
		// n1 = (n1+n2)decel / (accel + decel)
		srdX->accel_stop = (unsigned int) ((float)step*decel) / ((float)accel+decel);
		// We must accelerate at least 1 step before we can start deceleration.
		if(srdX->accel_stop == 0){
			srdX->accel_stop = 1;
		}

		// Use the limit we hit first to calc decel.
		if(srdX->accel_stop <= max_s_lim){
			// no run period
			srdX->decel_val = srdX->accel_stop - step;
		}
		else{
			srdX->decel_val = (signed int) -(max_s_lim*accel/decel);
			//srdX->accel_stop = (unsigned int) powf(speed,2.0f)/(0.736f*ALPHA*accel); // ---------------- test Linear factor --------------
		}
		// We must decelrate at least 1 step to stop.
		if(srdX->decel_val == 0){
			srdX->decel_val = -1;
		}

		// Find step to start decleration.
		srdX->decel_start = step + srdX->decel_val;

		// If the maximum speed is so low that we dont need to go via accelration state.
		if(srdX->step_delay <= srdX->min_delay){
			srdX->step_delay = srdX->min_delay;
			srdX->run_state = RUN;
		}
		else{
			srdX->run_state = ACCEL;
		}

		// Reset counter.
		srdX->accel_count = 0;
		srdX->running = 1;
		//OCR1A = 10;
		// Set Timer
		TIM_ITConfig(TIM3, srdX->channel , ENABLE);
	}
}

/*! @brief Move the stepper motor
 *
 *  Makes the stepper motor move
 *  It accelrate with given accelration up to maximum speed and decelerate
 *  with given deceleration so it stops at the given step.
 *  If accel/decel is to small and steps to move is to few, speed might not
 *  reach the max speed limit before deceleration starts.
 *
 *	@param *srdX  Pointer to speedRampData structure.
 *  @param accel  Accelration to use, in rad/sec^2.
 *  @param decel  Decelration to use, in rad/sec^2.
 *  @param speed  Max speed, in rad/sec.
 */
void speed_cntr_Speed(speedRampData *srdX, float accel, float decel, float speed)
{
	//! Number of steps before we hit max speed.
	int currSpeed, stepNb;

	if (srdX->run_state == STOP){
		if(speed!=0){
			srdX->running = 1;
			// Set accelration by calc the first (c0) step delay .
			// step_delay = 0.676 * f * sqrt(2*alpha/accel)
			srdX->step_delay = (unsigned int) (T_FREQ_148 * sqrtf((float) A_x2 / accel));
			stepNb = (int) (powf((speed),2.0f) / ((float) A_x2 * accel));
			srdX->min_delay = (unsigned int) (A_T / speed);
			srdX->accel_count = 0;
			srdX->decel_start = stepNb * 2;
			srdX->run_state = ACCEL;
			TIM_ITConfig(TIM3, srdX->channel , ENABLE);
		}
	} else {
		// Compute current speed

		currSpeed = (A_T / srdX->step_delay);
		if ( speed == 0 && currSpeed < MIN_SPEED){ // If speed too low
			srdX->run_state = STOP;
		} else if(currSpeed < speed-1) {
			stepNb = (int) (powf((speed-currSpeed),2.0f) / ((float) A_x2 * accel));
			srdX->min_delay = (unsigned int) (A_T / speed);

			if(srdX->run_state != ACCEL){
				srdX->accel_count = 0;
				srdX->run_state = ACCEL;
			}
		} else if(currSpeed > speed+1) {
			// How many step needed to get to speed
			stepNb = (int) (powf((currSpeed-speed),2.0f) / ((float) A_x2 * decel));
			if(speed>=MIN_SPEED){
				// Set new speed limit, by calc min_delay to use in timer.
				// min_delay = (alpha * t_freq) / w
				srdX->min_delay = (unsigned int) (A_T / speed);
			} else {
				srdX->min_delay = 0;
			}
			if (srdX->run_state != DECEL){
				srdX->accel_count = -stepNb;
				// ...in DECEL state.
				srdX->run_state = DECEL;
			}
		}
	}

}

void speed_cntr_Goto(float Cx, float Cy, float Cz, float Ce,float speed)
{
	float dx,dy,dz,de;
	float nPas;

	dx = fabsf(Cx-mPos.x);
	dy = fabsf(Cy-mPos.y);
	dz = fabsf(Cz-mPos.z);
	de = fabsf(Ce-mPos.e);

	stGen.run_mode = STEP;

	if(AlmostEqual(Cx,mPos.x) && AlmostEqual(Cy,mPos.y) && AlmostEqual(Cz,mPos.z) && AlmostEqual(Ce,mPos.e)){
		UB_USB_CDC_SendString("END",LFCR);
	} else {
		if (dx > dy && dx > dz && dx > de) {

			sFactor.x = 1.0f;
			sFactor.y = dy/dx;
			sFactor.z = dz/dx;
			sFactor.e = de/dx;

			nPas = dx;


		} else if (dy > dz && dy > de) {

			sFactor.x = dx/dy;
			sFactor.y = 1.0f;
			sFactor.z = dz/dy;
			sFactor.e = de/dy;

			nPas = dy;

		} else if (dz > de) {

			sFactor.x = dx/dz;
			sFactor.y = dy/dz;
			sFactor.z = 1.0f;
			sFactor.e = de/dz;

			nPas = dz;


		} else {

			sFactor.x = dx/de;
			sFactor.y = dy/de;
			sFactor.z = dz/de;
			sFactor.e = 1.0f;

			nPas = de;


		}
		speed_cntr_Step(&stGen,(int)nPas,min4Accel(mPara,sFactor), min4Decel(mPara,sFactor),min4Speed(mPara,sFactor));
	}

	if (Cx < mPos.x) setDir(DOUT_DIR1,BACKWARD);
	else setDir(DOUT_DIR1,FORWARD);
	if (Cy < mPos.y) setDir(DOUT_DIR2,BACKWARD);
	else setDir(DOUT_DIR2,FORWARD);
	if (Cz < mPos.z) setDir(DOUT_DIR3,BACKWARD);
	else setDir(DOUT_DIR3,FORWARD);
	if (Ce < mPos.e) setDir(DOUT_DIR4,BACKWARD);
	else setDir(DOUT_DIR4,FORWARD);


	mPos.x = Cx;
	mPos.y = Cy;
	mPos.z = Cz;
	mPos.e = Ce;
}

unsigned int speed_cntr_STOP(void)
{

	stGen.run_mode = SPEED;
	speed_cntr_Speed(&stGen,SMAX_ACCEL,SMAX_DECEL,0);
	while(stGen.running);
	return stGen.step_count;
}

/*! @brief Init of Timer.
 *
 *  Set up Timer.
 */
void speed_cntr_Init(void)
{

	stGen.channel = TIM_IT_CC1;
	// Tells what part of speed ramp we are in.
	stGen.run_state = STOP;
	stGen.step_count = 0;
	stGen.rest = 0;
	stGen.do_delay = 0;


	mPos = (machineAxis){0,0,0,0};
	sFactor = (machineAxis){0,0,0,0};
	sCount = (machineAxis){0,0,0,0};


	NVIC_InitTypeDef NVIC_InitStructure;

	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	/* Enable the TIM3 gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // Highest priority
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* -----------------------------------------------------------------------
    TIM3 Configuration: Output Compare Timing Mode:
    CC1 update rate = TIM3 counter clock / CCR1_Val
  ----------------------------------------------------------------------- */


	/* Compute the prescaler value */
	//PrescalerValue = (uint16_t) ((SystemCoreClock) / 72000000) - 1;
	PrescalerValue = (uint16_t) ((SystemCoreClock) / 1000000) - 1; // 1MHZ Clock


	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 65535;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	/* Prescaler configuration */
	TIM_PrescalerConfig(TIM3, PrescalerValue, TIM_PSCReloadMode_Immediate);

	/* Output Compare Timing Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	//TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM3, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable);

	/* Output Compare Timing Mode configuration: Channel2 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	//TIM_OCInitStructure.TIM_Pulse = CCR2_Val;
	TIM_OCInitStructure.TIM_Pulse = 0;

	TIM_OC2Init(TIM3, &TIM_OCInitStructure);

	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Disable);

	/* TIM Interrupts enable */
	//TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2 , ENABLE);

	/* TIM3 enable counter */
	TIM_Cmd(TIM3, ENABLE);

}

/**
 * @brief  This function handles TIM3 global interrupt request.
 * @param  None
 * @retval None
 */
void TIM3_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);

		capture = TIM_GetCapture1(TIM3);
		if(stGen.do_delay){
			stGen.do_delay = 0;
			pulseStepD(&stGen);
			ttemp1 = stGen.step_delay - ttemp1;
		} else {
			ttemp1 = speed_cntr_interrupt(&stGen);
			UB_DigOut_Toggle(LED_GREEN);
		}
		TIM_SetCompare1(TIM3, capture + ttemp1); // call speed_cntr
	}
}

/*! @brief Timer/Counter1 Output Compare A Match Interrupt.
 *
 *  Timer/Counter1 Output Compare A Match Interrupt.
 *  Increments/decrements the position of the stepper motor
 *  exept after last position, when it stops.
 *  The \ref step_delay defines the period of this interrupt
 *  and controls the speed of the stepper motor.
 *  A new step delay is calculated to follow wanted speed profile
 *  on basis of accel/decel parameters.
 */
unsigned int speed_cntr_interrupt( speedRampData *srdX ){

	// Holds next delay period.
	unsigned int temp_step_delay = 0;


	switch(srdX->run_state) {
	case STOP:
		srdX->step_count = 0;
		srdX->rest = 0;
		sync_val = 0.0f;
		// Stop Timer
		TIM_ITConfig(TIM3, srdX->channel , DISABLE);
		srdX->running = 0;
		UB_USB_CDC_SendString("END",LFCR);
		break;

	case ACCEL:
		// --- OUTPUT STEP ---

		pulseStepH(srdX);

		srdX->step_count++;
		srdX->accel_count++;

		calcNStep(srdX);

		// Check if we should start deceleration.
		if((srdX->step_count >= srdX->decel_start) && srdX->run_mode != SPEED ) {
			srdX->accel_count = srdX->decel_val;
			srdX->run_state = DECEL;
		}
		// Check if we hitted max speed.
		else if(srdX->step_delay <= srdX->min_delay) {
			srdX->last_accel_delay = srdX->step_delay;
			srdX->step_delay = srdX->min_delay;
			srdX->rest = 0;
			srdX->run_state = RUN;
		}

		temp_step_delay = 8;
		break;

	case RUN:
		// --- OUTPUT STEP ---

		pulseStepH(srdX);

		srdX->step_count++;

		// Chech if we should start decelration.
		if(srdX->step_count > srdX->decel_start && srdX->run_mode != SPEED ) {
			srdX->accel_count = srdX->decel_val;
			// Start decelration with same delay as accel ended with.
			srdX->step_delay = srdX->last_accel_delay;
			srdX->run_state = DECEL;
		}

		temp_step_delay = 9;
		break;

	case DECEL:
		// --- OUTPUT STEP ---

		pulseStepH(srdX);

		srdX->step_count++;
		srdX->accel_count++;

		calcNStep(srdX);

		// Check if we at last step
		if(srdX->accel_count >= 0) {
			if(srdX->step_delay >= srdX->min_delay && srdX->min_delay != 0  && srdX->run_mode == SPEED) {
				srdX->last_accel_delay = srdX->step_delay;
				srdX->step_delay = srdX->min_delay;
				srdX->rest = 0;
				srdX->run_state = RUN;
			} else
				srdX->run_state = STOP;
		}

		temp_step_delay = 8;
		break;
	}
	srdX->do_delay = 1;
	return temp_step_delay;
}


/* Private functions ---------------------------------------------------------*/

/*! @brief Compute next step
 *
 *	Handle calculation for next step step delay
 *
 *	@param *srdX  Pointer to speedRampData structure.
 */
static void calcNStep( speedRampData *srdX ){

	unsigned int new_step_delay;

	new_step_delay = srdX->step_delay - (((2 * (long)srdX->step_delay) + srdX->rest)/(4 * srdX->accel_count + 1));
	srdX->rest = ((2 * (long)srdX->step_delay)+srdX->rest)%(4 * srdX->accel_count + 1);

	srdX->step_delay = new_step_delay;
}

/*! @brief Set step pin high
 *
 *  Handle hardware pulse generation
 *
 *	@param *srdX  Pointer to speedRampData structure.
 */
static void pulseStepH(){

	if(AlmostEqual(sFactor.x,1)) {
		UB_DigOut_Lo(DOUT_ST1); UB_DigOut_Toggle(LED_GREEN);
	} else {
		sCount.x += sFactor.x;
		if (sCount.x >= 1.0f){
			sCount.x -= 1.0f;
			UB_DigOut_Lo(DOUT_ST1); UB_DigOut_Toggle(LED_GREEN);
		}
	}
	if(AlmostEqual(sFactor.y,1)) {
		UB_DigOut_Lo(DOUT_ST2);  UB_DigOut_Toggle(LED_ORANGE);
	} else {
		sCount.y += sFactor.y;
		if (sCount.y >= 1.0f){
			sCount.y -= 1.0f;
			UB_DigOut_Lo(DOUT_ST2);  UB_DigOut_Toggle(LED_ORANGE);
		}
	}
	if(AlmostEqual(sFactor.z,1)) {
		UB_DigOut_Lo(DOUT_ST3); UB_DigOut_Toggle(LED_RED);
	} else {
		sCount.z += sFactor.z;
		if (sCount.z >= 1.0f){
			sCount.z -= 1.0f;
			UB_DigOut_Lo(DOUT_ST3); UB_DigOut_Toggle(LED_RED);
		}
	}
	if(AlmostEqual(sFactor.e,1)) {
		UB_DigOut_Lo(DOUT_ST4);  UB_DigOut_Toggle(LED_BLUE);
	} else {
		sCount.e += sFactor.e;
		if (sCount.e >= 1.0f){
			sCount.e -= 1.0f;
			UB_DigOut_Lo(DOUT_ST4);  UB_DigOut_Toggle(LED_BLUE);
		}
	}

}
/*! @brief Set step pin down
 *
 *  Handle hardware pulse generation
 *
 *	@param *srdX  Pointer to speedRampData structure.
 */
static void pulseStepD(){

	UB_DigOut_Hi(DOUT_ST1);
	UB_DigOut_Hi(DOUT_ST2);
	UB_DigOut_Hi(DOUT_ST3);
	UB_DigOut_Hi(DOUT_ST4);

}

/*! @brief Set dir pin
 *
 *  Handle hardware direction output
 *
 *	@param *srdX  Pointer to speedRampData structure.
 *  @param dir  Motor direction.
 */
void setDir(DOUT_NAME_t p, MVT_DIR_t dir) {
	if (dir == BACKWARD) UB_DigOut_Lo(p);
	else UB_DigOut_Hi(p);
}


bool AlmostEqual(float A, float B)
{
	int maxUlps = 5;
    // Make sure maxUlps is non-negative and small enough that the
    // default NAN won't compare as equal to anything.
    //assert(maxUlps > 0 && maxUlps < 4 * 1024 * 1024);
    int aInt = *(int*)&A;
    // Make aInt lexicographically ordered as a twos-complement int
    if (aInt < 0)
        aInt = 0x80000000 - aInt;
    // Make bInt lexicographically ordered as a twos-complement int
    int bInt = *(int*)&B;
    if (bInt < 0)
        bInt = 0x80000000 - bInt;
    int intDiff = fabsf(aInt - bInt);
    if (intDiff <= maxUlps)
        return true;
    return false;
}

float min4Speed(MOTOR_PARAM_t * mots, machineAxis f){
	float sx = mots[0].max_speed/f.x ;
	float sy = mots[1].max_speed/f.y ;
	float sz = mots[2].max_speed/f.z ;
	float se = mots[3].max_speed/f.e ;

	float s1 = ((sx < sy)? sx : sy);
	float s2 = ((sz < se)? sz : se);

	return ((s1 < s2)? s1 : s2);
}
float min4Accel(MOTOR_PARAM_t * mots, machineAxis f){
	float sx = mots[0].accel/f.x ;
	float sy = mots[1].accel/f.y ;
	float sz = mots[2].accel/f.z ;
	float se = mots[3].accel/f.e ;

	float s1 = ((sx < sy)? sx : sy);
	float s2 = ((sz < se)? sz : se);

	return ((s1 < s2)? s1 : s2);
}
float min4Decel(MOTOR_PARAM_t * mots, machineAxis f){
	float sx = mots[0].decel/f.x ;
	float sy = mots[1].decel/f.y ;
	float sz = mots[2].decel/f.z ;
	float se = mots[3].decel/f.e ;

	float s1 = ((sx < sy)? sx : sy);
	float s2 = ((sz < se)? sz : se);

	return ((s1 < s2)? s1 : s2);
}
