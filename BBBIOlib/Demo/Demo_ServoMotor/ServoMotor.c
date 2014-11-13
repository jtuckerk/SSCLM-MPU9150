
#include <stdio.h>
#include <stdlib.h>
#include "../../BBBio_lib/BBBiolib.h"

/* Servo values for TG9 servos: */
/* Servo 0 degree angle pulse high time in msec */
#define SRV_0    0.45
/* Servo 180 degree angle pulse high time in msec */
#define SRV_180  2.45

/* Pulse repetition frequency in Hz */
#define FRQ 50.0f
/* Pulse period in msec */
#define PER (1.0E3/FRQ)


/* ----------------------------------------------------------- */
int main(void)
{
	iolib_init();       /* Initialize I/O library - required */
	int i;
	float SM_1_duty ;	/* Servomotor 1 , connect to ePWM0A */

	for(i = 0 ; i <= 180 ; i += 10)
	{
	    /* Calculate duty cycle */
		/* Note: the 100-X duty cyle is to account for the level shifter that inverts */
		SM_1_duty = 100.0 - ((SRV_0/PER) + (i/180.0) * ((SRV_180-SRV_0)/PER))*100.0;
		printf("Angle : %d , duty : %f\n" ,i ,SM_1_duty);
	    BBBIO_PWMSS_Setting(BBBIO_PWMSS0 , FRQ, SM_1_duty , SM_1_duty); /* Set up PWM */
		BBBIO_ehrPWM_Enable(BBBIO_PWMSS0); /* Enable PWM, generate waveform */
		sleep(2); /* Allow time for servo to settle and for humans to see something. */
		BBBIO_ehrPWM_Disable(BBBIO_PWMSS0); /* Disable PWM, stop generating waveform */
	}

	iolib_free();     /* Release I/O library prior to program exit - required */
	return(0);
}


