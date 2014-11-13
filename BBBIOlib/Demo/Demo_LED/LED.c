
#include <stdio.h>
#include <stdlib.h>
#include "../../BBBio_lib/BBBiolib.h"

int
main(void)
{
	int del;
	iolib_init(); /* Initialize I/O library - required */
	iolib_setdir(8,11, BBBIO_DIR_IN);   /* Set pin P8 - 11 as input */
	iolib_setdir(8,12, BBBIO_DIR_OUT);  /* Set pin P8 - 12 as output */

	int count = 0;
	while(count < 50)
	{
		count ++ ;
		if (is_high(8,11))   /* Check if in is high (i.e. button pressed) */
		{
			del=100; /* fast speed */
		}
		if (is_low(8,11))    /* Check if in is low (i.e button is released) */
		{
			del=500; /* slow speed */
		}

		pin_high(8,12);       /* Set pin to high - LED on */
		iolib_delay_ms(del);  /* Delay for 'del' msec */
		pin_low(8,12);		  /* Set pin to low - LED off */
		iolib_delay_ms(del);  /* Delay for 'del' msec */

	}
	iolib_free();  /* Release I/O library prior to program exit - required */
	return(0);
}


