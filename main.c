/* ----------------------------------------------------------------------------
 * File    : main.c
 * Author  : Denis Jullien
 * Version : V1.0
 * Date    : 08/10/2014
 *
 *  Simple Gcode interpreter for 4axis stepper control
 *
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * Denis Jullien wrote this file. As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return.
 * ----------------------------------------------------------------------------
 */

#include "main.h"
#include <stdlib.h>
#include "stepper_drv.h"
#include "string2.h"

#define CMD_BUF_SIZE 32

extern speedRampData stGen;
extern machineAxis mPos;

int main(void)
{
	char buf[APP_TX_BUF_SIZE];
	uint8_t done = 0;
	USB_CDC_RXSTATUS_t check=RX_EMPTY;

	SystemInit();

	UB_DigIn_Init();
	UB_DigOut_Init();
	UB_Systick_Init();
	UB_USB_CDC_Init();  // Init of USB-OTG-Port as CDC-Device

	speed_cntr_Init();

	while(1)
	{
		if(UB_USB_CDC_GetStatus()==USB_CDC_CONNECTED) {
			check=UB_USB_CDC_ReceiveString(buf);
			if(check==RX_READY) {

				UB_USB_CDC_SendString(buf,LFCR);
				done = 0;
			}
		}
		if (!stGen.running && !done) {
			doMove(buf);
			done = 1;
		}
	}
}

DECODE_STATUS_t doMove (char *ptr) {
	char * cmd, *c[4];
	float d[4];
	char listAxes[] = {'X', 'Y', 'Z', 'E'};
	uint8_t i,l;

	cmd = strtok(ptr, " ");
	for(i=0; i<4; ++i){
		c[i] = strtok(NULL, " ");
	}

	d[0] = mPos.x;
	d[1] = mPos.y;
	d[2] = mPos.z;
	d[3] = mPos.e;

	switch (cmd[0]) {
		case 'G':
			i=0;
			for (l = 0; l < 4; l++) {
				if (c[i] != NULL && listAxes[l]==c[i][0]){
					d[l] = atof(c[i]+1);
					i++;
				}
			}
			switch (cmd[1]) {
				case '0':
					UB_USB_CDC_SendString("G0 Rapid Linear Motion",LFCR);
					speed_cntr_Goto(d[0],d[1],d[2],d[3],40);
					break;
				case '1':
					UB_USB_CDC_SendString("G1 Linear Motion",LFCR);
					speed_cntr_Goto(d[0],d[1],d[2],d[3],20);
					break;
				default:
					return FAIL;
					break;
			}
			break;
		default:
			return FAIL;
			break;
	}
	return OK;
}
