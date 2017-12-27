/*
 * playback.cpp
 *
 *  Created on: Aug 27, 2017
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

	FILE* stream = fopen("pos_output.csv", "r");

	int c = 0;
	char line[1024];
	while (fgets(line, 1024, stream))
	{
		sscanf(line, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &state->q[0], &state->q[1], &state->q[2], &state->q[3], &state->q[4], &state->q[5], &state->q[6], &state->q[7], &state->q[8]);

		state_array[c] = *state;
		c++;

		// NOTE strtok clobbers tmp

	}

	int max_frames = c;
	c=0;
	while (true)
	{
		if (c >= max_frames)
			c = 0;


		forward(s, &state_array[c]);
		vis_draw(v, s, true);
		c++;
	}

	vis_close(v);

	return 0;
}
