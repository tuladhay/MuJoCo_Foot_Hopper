
#include <stdio.h>
#include "stdlib.h"
#include <unistd.h>
#include "slip.h"
#include "mujoco.h"
#include "glfw3.h"

typedef struct slip_t slip_t;


int main()
{

	slip_t* s = init(".");

	slip_vis_t* v = vis_init();

	state_t* state = malloc(sizeof(state_t));

	for (int i = 0; i < nQ; i++)
	{
		state->q[i] = 0.0;
		state->qd[i] = 0.0;
		state->qdd[i] = 0.0;
	}
	for (int i = 0; i < nU; i++)
		state->u[i] = 0.0;

	state->q[1] = 0.7; //Height off the ground
	state->q[3] = 0.45; //Resting Leg Length

	//int ret = get_stationary_init(s, state);
	if ( false) //ret
	{
		printf("error couldn't set desired root height!");
		vis_close(v);
		return -1;
	}

	printf("State:\n");
	for (int i = 0; i < nQ; i++)
		printf("%f\t", state->q[i]);
	printf("\n");

	printf("Motor forces:\n");
	for (int i = 0; i < nU; i++)
		printf("%f\t", state->u[i]);
	printf("\n");

	forward(s, state);

	for (int i = 0; i < nQ; i++)
		printf("%f\t", state->qdd[i]);
	printf("\n");

// 	while (true)
// 	{

// 		vis_draw(v, s, false);
// //		forward(s, state);
// 		mjtNum simstart = s->d->time;
		
// 		while( s->d->time - simstart < 1.0/60.0 )
// 		{
//         	step(s, 0);
// 		}


// 	}

	// for (int i = 0; i < 2000; i++)
	// {	
	// 	printf("\nTimestep: %d,  height: %f ", i, state->q[1]);
	// 	step(s, 0);
	// 	vis_draw(v, s, false);
	// }


	while (true)
	{
		vis_draw(v, s, false);
        step(s, 500.0);
	}


	vis_close(v);

	return 0;
}
