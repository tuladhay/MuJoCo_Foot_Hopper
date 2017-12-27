/*
 * main_joy.c
 *
 *  Created on: Aug 29, 2017
 *      Author: tapgar
 */



#include "slip.h"
#include <stdio.h>
#include "stdlib.h"
#include <unistd.h>
#include <string.h>


int main()
{

	slip_t* s = init();

	slip_vis_t* v = vis_init();

	state_t* state = malloc(sizeof(state_t));
	state_t state_array[100];

	int c = 0;
	while (true)
	{
		step_ctrl(s);
		if (c++ > 30)
		{
			vis_draw(v, s, false);
			c=0;
		}
	}

	vis_close(v);

	return 0;
}
