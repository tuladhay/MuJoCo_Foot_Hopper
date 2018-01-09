#include <stdio.h>
#include <stdbool.h>
#include "mujoco.h"
#include "slip.h"
#include <stddef.h>
#include <stdlib.h>
#include <math.h>
#include "glfw3.h"
#include "stdlib.h"
#include "string.h"
/*
This program is meant to include functionality of a basic raibert hopper, while having
all the functions necessary to interact with MuJoCo. So it is all in one place so that
the raibert controller can be loaded as a library.
*/

static bool m_bMJActivated = false;
static bool glfw_initialized = false;

static bool initialize = true;
bool flag = false;

// See the XML file for the DOF/joint details
#define rootx 0
#define rootz 1
#define leg_tau 2		// actuator here
#define leg_motor 3		// actuator here
#define spring 4		// toe
#define foot_joint 5	// actuator here

#define compression 1
#define thrust 2
#define flight 3

#define input_leg_tau 0
#define input_leg_motor 1
#define input_foot_joint 2


typedef struct slip_t {
        mjData *d;
}slip_t;

struct slip_vis_t {
	GLFWwindow* window;
	mjvCamera cam;
	mjvOption opt;
	mjvScene scn;
	mjrContext con;
};

static mjModel* m;

slip_t* init(const char *basedir)
{
    if (!m_bMJActivated) {
        // If no base directory is provided, use . as the base directory
		const char thisdir[] = ".";
		if (!basedir)
			basedir = thisdir;
		size_t basedirlen = strnlen(basedir, 4096);

		// Construct key file path
		char *keyfile;
		const char keyfilename[] = "mjkey.txt";
		keyfile = malloc(basedirlen + 1 + sizeof keyfilename);
		strncpy(keyfile, basedir, basedirlen);
		keyfile[basedirlen] = '/';
		strcpy(keyfile + basedirlen + 1, keyfilename);// Construct XML file path
		char *xmlfile;
		const char xmlfilename[] = "slip_with_foot.xml";
		xmlfile = malloc(basedirlen + 1 + sizeof xmlfilename);
		strncpy(xmlfile, basedir, basedirlen);
		xmlfile[basedirlen] = '/';
		strcpy(xmlfile + basedirlen + 1, xmlfilename);

		// Activate mujoco and load the model
		mj_activate(keyfile);
		char error[1000] = "Could not load XML model";
		m = mj_loadXML(xmlfile, 0, error, 1000);
		if (m) {
			m_bMJActivated = true;
		} else {
			mju_error_s("Load model error: %s", error);
		}

		// Free allocated strings
		free(keyfile);
		free(xmlfile);
    }
	m_bMJActivated = true;

    slip_t *s = calloc(1, sizeof(slip_t));
    s->d = mj_makeData(m); // d is a pointer to mjData
	// Once both mjModel and mjData are allocated and initialized, we can call the various simulation functions. 

    return s;
}

void forward(slip_t* s, state_t* state)
{

	mju_zero(s->d->xfrc_applied, m->nbody*6); //clear applied forces
	//wipe out data and replace with state info
	for (int i = 0; i < nQ; i++)
	{
		s->d->qpos[i] = state->q[i];
		s->d->qvel[i] = state->qd[i];
	}

	for (int i = 0; i < nU; i++)
		s->d->ctrl[i] = state->u[i];


	mju_zero(s->d->qacc, m->nv);
	mju_zero(s->d->qacc_warmstart, m->nv);
	mju_zero(s->d->qfrc_applied, m->nv);

	mj_forward(m, s->d);

	mj_forward(m, s->d);

	// extra solver iterations to improve warmstart (qacc) at center point
	for( int rep=1; rep<3; rep++ )
		mj_forwardSkip(m, s->d, mjSTAGE_VEL, 1);

	for (int i = 0; i < nQ; i++)
	{
		state->qdd[i] = s->d->qacc[i];
	}
}

void step(slip_t* s, double action)
{	
	if (initialize)						// initialize set to False inside controller
	{
		set_initial_state(s);
	}
	s->d->ctrl[2] = action;				// Set by RL
	
	mj_step(m, s->d);					// pass the mjModel and mjData
	controller(s);						// Sets s->d->ctrl[0] = u[0]; And for u[1]
}

void set_state(slip_t* s, state_t* state)
{
	for (int i = 0; i < nQ; i++)
	{
		s->d->qpos[i] = state->q[i];
		s->d->qvel[i] = state->qd[i];
		s->d->qacc[i] = state->qdd[i];
	}
	s->d->time = state->t;
}

void run_forward(slip_t* s, state_t* state, double DT)
{
	//wipe out data and replace with state info
	for (int i = 0; i < nQ; i++)
	{
		s->d->qpos[i] = state->q[i];
		s->d->qvel[i] = state->qd[i];
	}

	for (int i = 0; i < nU; i++)
		s->d->ctrl[i] = state->u[i];

	mju_zero(s->d->qacc, m->nv);
	mju_zero(s->d->qacc_warmstart, m->nv);
	mju_zero(s->d->qfrc_applied, m->nv);
	mju_zero(s->d->xfrc_applied, m->nbody*6);

	mj_forward(m, s->d);

	int iters = (int)(DT/m->opt.timestep);

	for (int i = 0; i < iters; i++)
		mj_step(m, s->d);

	for (int i = 0; i < nQ; i++)
	{
		state->q[i] = s->d->qpos[i];
		state->qd[i] = s->d->qvel[i];
		state->qdd[i] = s->d->qacc[i];
	}
}

void step_ctrl(slip_t* s)
{
	mj_step1(m, s->d);
	int count;
	const float* axes = glfwGetJoystickAxes(GLFW_JOYSTICK_1, &count);
	for (int i = 0; i < nU; i++)
		s->d->ctrl[i] = 150*(double)(axes[i]);

	mj_step2(m, s->d);
}

void get_joint_limits(pos_limits_t *lim)
{
	for (int i = 0; i < nQ; i++)
	{
		if (!m->jnt_limited[i])
		{
			lim->lb[i] = -1e20;
			lim->ub[i] = 1e20;
		}
		else {
			lim->lb[i] = m->jnt_range[i*2];
			lim->ub[i] = m->jnt_range[i*2+1];
		}
	}
}

void get_motor_limits(motor_limits_t *lim)
{
	for (int i = 0; i < nU; i++)
	{
		if (!m->actuator_ctrllimited[i])
		{
			lim->lb[i] = -1e20;
			lim->ub[i] = 1e20;
		}
		else {
			lim->lb[i] = m->actuator_ctrlrange[i*2];
			lim->ub[i] = m->actuator_ctrlrange[i*2+1];
		}
	}
}

/**********************************************************************************
						OBSERVATION AND STEP FUNCTIONS FOR RL ENV
***********************************************************************************
*/
// This function should return the necessary state that will be used as RL inputs.
// The return type should be an array of doubles since that is what rllab will take.
void get_observation(slip_t* s, double* state_vec)		// void because we want to use pointers?
{	// state_vec is a double array passed from env.
	state_vec[0] = s->d->qpos[rootx];			// x pos
	state_vec[1] = s->d->qpos[rootz];			// z pos
	state_vec[2] = s->d->qvel[rootx];			// x vel
	state_vec[3] = s->d->qvel[rootz];			// z vel
	state_vec[4] = s->d->qpos[leg_tau];			// leg tau pos
	state_vec[5] = s->d->qpos[foot_joint];			// foot joint motor pos
	state_vec[6] = s->d->site_xpos[2];		// foot heel Z pos
	state_vec[7] = s->d->site_xpos[5];		// foot toe Z pos
}


/**********************************************************************************
									INITIALIZE
***********************************************************************************
*/
void set_initial_state(slip_t* s)
{
	for (int i = 0; i < nQ; i++)
	{
		s->d->qpos[i] = 0;
		s->d->qvel[i] = 0;
		s->d->qacc[i] = 0;
	}
	s->d->time = 0;

	s->d->qpos[rootz] = 1.2;
	s->d->qpos[leg_motor] = 0.45;
	// dont know what cpos are; to get those, you can just run one step
}


/***********************************************************************************
************************************************************************************
									CONTROLLER
************************************************************************************
*/
void controller(slip_t* s)
{
	double des_velocity = 2;			// m/s
	double gain_Kp_L0 = 6000;
	double gain_Kd_L0 = 200;
	double gain_Kp_swing = 2500;
	double gain_Kd_swing = 150;
	double gain_footDisp = 0.14;

	//double gain_Kp_hip = 2000;
	//double gain_Kd_hip = 200;

	double L_flight = 0.45;
	double L_extension = 0.55;
	double xf_Point = 0;
	double rest_leg_length = 1.025;
	bool toe_bias = true;

	// These static variables should retain their values between function calls
	static int dynamic_state = 3;
	static double des_td_angle = 0;
	static double touchdown_time = 0;
	static double stance_time = 0;
	static double apex_velocity = 0;


	state_t state;

	if (initialize)
	{
		for (int i = 0; i < nQ; i++)
		{
			state.q[i] = 0;
			state.qd[i] = 0;
			state.qdd[i] = 0;
		}
		state.t = 0;
		state.cpos[0] = 0;		// foot heel z,
		state.cpos[1] = 0;		// foot toe z

		initialize = false;
	}

	
	for (int i = 0; i < nQ; i++)
	{
		state.q[i] = s->d->qpos[i];
		state.qd[i] = s->d->qvel[i];
		state.qdd[i] = s->d->qacc[i];
	}
	state.t = s->d->time;
	state.cpos[0] = s->d->site_xpos[2];		// foot heel z,
	state.cpos[1] = s->d->site_xpos[5];		// foot toe z

	// 1 = compression
	// 2 = thrust
	// 3 = flight

	// Check for the current dynamic state
	// Then use switch case to implement controller depending
	// on the dynamic state
	switch(dynamic_state)
	{
		case 1 :
			// COMPRESSION to Thrust
			// check if body acceleration is greater than 0.0005
			// if body qdd > 0.005, dyn_state = 2, i.e THRUST
			// we can also try to do the same with:
			// Transition to thrust if leg (spring) begins to extend
			// if (state->qd[1] > 0.0)	// body z velocity
			// Dont know which one is preferrable. I think spring can have oscillations and body z could be actually better.
			if (state.qd[spring] < 0.0)		// spring z velocity
			{
				dynamic_state = 2;
				break;
			}
			break;

		case 2 :
			// THRUST to Flight
			// If foot contacts are above the ground, then dyn_state = FLIGHT
			if (  (state.cpos[0] > 0.01)   &&   (state.cpos[1] > 0.01))
			{
				dynamic_state = 3;
				stance_time = s->d->time - touchdown_time;

				// Calculate the desired touchdown angle depending on whether you want to account for toe bias
				// Adding bias reduces velocity tracking error. I think this is because due to the added foot, the toe touched the ground first, however,
				// the Raibert Controller assumes that it is the mid of the foot (point feet)
				if (toe_bias)
				{
					xf_Point = (0.5*(state.qd[rootx])*stance_time) + gain_footDisp*(apex_velocity - des_velocity)  -  0.01;		// Toe touchdown bias
				}
				else
				{
					xf_Point = (0.5*(state.qd[rootx])*stance_time) + gain_footDisp*(apex_velocity - des_velocity);				// NO BIAS
				}
				
				des_td_angle = asin(xf_Point/rest_leg_length);

				if (des_td_angle > 0.25)		// approx 20 degrees
					des_td_angle = 0.25;

				if (des_td_angle < -0.25)
					des_td_angle = -0.25;

				dynamic_state = 3;

				flag = true;	// flag to update the apex velocity
				break;
			}
			break;


		case 3 :
			// FLIGHT to Compression
			// check if body acceleration is some negative threshold,
			// and there is ground contact then set state to compression


			if (state.qd[rootz] < 0 && flag)			// when it just crosses apex
			{
				apex_velocity = state.qd[rootx];
				flag = false;
			}


			//if (   (state->qd[1] < 0)   &&   ((state->cpos[0] == 0) || (state->cpos[1] == 0))   )
			if (  ((state.cpos[0] < 0) || (state.cpos[1] < 0))   )
			{
				dynamic_state = 1;

				// Update the flight time
				touchdown_time = s->d->time;
				break;
			}
			break;
	}

	// FOR TESTING ONLY **********************************************
	// state->des_td_angle = 0.5;
	// state->dynamic_state = 2;

	// ***************************************************************
	// Implement Force and Torque Controllers based on current state
	// u[0] is teg_tau
	// u[1] is leg_motor
	// u[2] is foot_joint

	switch(dynamic_state)
	{
		case 1 :	// Compression
		// fixate the leg motor
		//state->u[0] =  -gain_Kp_hip*(state->q[3] - state->des_td_angle) - gain_Kp_hip*state->qd[3];
		u[input_leg_tau] = 0;
		u[input_leg_motor] = (-gain_Kp_L0*(state.q[leg_motor] - L_flight) - gain_Kd_L0*state.qd[leg_motor]);
		//u[2] = 0;

		break;

		case 2 :	// Thrust
		//state->u[0] = -gain_Kp_hip*(state->q[3] - state->des_td_angle) - gain_Kp_hip*state->qd[3];
		u[input_leg_tau] = 0;
		u[input_leg_motor] = -gain_Kp_L0*(state.q[leg_motor]- L_extension) - gain_Kd_L0*state.qd[leg_motor];
		//u[2] = 0;
		break;

		case 3 :	// Flight
		//state->des_td_angle = -0.3;
		u[input_leg_tau] = -gain_Kp_swing*(state.q[leg_tau] - des_td_angle) - gain_Kd_swing*state.qd[leg_tau];	// Swing leg
		u[input_leg_motor] = -gain_Kp_L0*(state.q[leg_motor] - L_flight) - gain_Kd_L0*state.qd[leg_motor];
		//u[2] = 0;
		//state->u[0] = -50;
		break;
	}

	//Setting the control values
	for (int i = 0; i < nU-1; i++)
		s->d->ctrl[i] = u[i];

}



/************************************************************************************
*************************************************************************************
				UTILITY FUNCTIONS FOR MOUSE, KEYBOARD AND VISUALIZATION
*************************************************************************************
*/

static void window_close_callback(GLFWwindow* window)
{
    vis_close(glfwGetWindowUserPointer(window));
}

static void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
	Scroll(xoffset, yoffset, glfwGetWindowUserPointer(window));
}
static void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
	MouseMove(xpos, ypos, glfwGetWindowUserPointer(window));
}
static void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
	MouseButton(button, act, mods, glfwGetWindowUserPointer(window));
}

static void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
	Keyboard(key, scancode, act, mods, glfwGetWindowUserPointer(window));
}


slip_vis_t* vis_init()
{
    // Check if model has been loaded
    if (!m) {
        mju_error("init must be called before vis_init");
        return NULL;
    }

    // Initialize GLFW if this is the first visualization window
    if (!glfw_initialized) {
        if (!glfwInit()) {
            mju_error("Could not initialize GLFW");
            return NULL;
        }
        glfw_initialized = true;
    }

    // Allocate visualization structure
    slip_vis_t *v = malloc(sizeof (slip_vis_t));

    // Create window
    v->window = glfwCreateWindow(1200, 900, "Slip", NULL, NULL);
    glfwMakeContextCurrent(v->window);
    glfwSwapInterval(1);

    // Set up mujoco visualization objects
    v->cam.lookat[0] = m->stat.center[0];
	v->cam.lookat[1] = m->stat.center[1];
	v->cam.lookat[2] = 0.8 + m->stat.center[2];
	v->cam.type = mjCAMERA_FREE;

	v->cam.distance = 0.5 * m->stat.extent;
	mjv_moveCamera(m, mjMOUSE_ROTATE_H, 0.75, 0.0, &v->scn, &v->cam);

    mjv_defaultOption(&v->opt);
    mjr_defaultContext(&v->con);
    mjv_makeScene(&v->scn, 1000);
    mjr_makeContext(m, &v->con, mjFONTSCALE_100);

    // Set callback for user-initiated window close events
    glfwSetWindowUserPointer(v->window, v);
    glfwSetWindowCloseCallback(v->window, window_close_callback);
	glfwSetCursorPosCallback(v->window, mouse_move);
	glfwSetMouseButtonCallback(v->window, mouse_button);
	glfwSetScrollCallback(v->window, scroll);
	glfwSetKeyCallback(v->window, keyboard);

    return v;
}

static bool bUserInput = false;

bool vis_draw(slip_vis_t *v, slip_t *s, bool bWaitUser)
{
	bool doOnce = true;

	while (bWaitUser || doOnce)
	{
		doOnce = false;
		// Return early if window is closed
		if (!v->window)
			return false;

		// Set up for rendering
		glfwMakeContextCurrent(v->window);
		mjrRect viewport = {0, 0, 0, 0};
		glfwGetFramebufferSize(v->window, &viewport.width, &viewport.height);

		// Render scene
		mjv_updateScene(m, s->d, &v->opt, NULL, &v->cam, mjCAT_ALL, &v->scn);
		mjr_render(viewport, &v->scn, &v->con);

		// Show updated scene
		glfwSwapBuffers(v->window);
		glfwPollEvents();

		if (bUserInput)
		{
			bUserInput = false;
			break;
		}

	}
	return true;
}

static bool button_left = false;
static bool button_middle = false;
static bool button_right =  false;
static double cursor_lastx = 0;
static double cursor_lasty = 0;


// mouse button
void MouseButton(int button, int act, int mods, slip_vis_t* v)
{
    // update button state
    button_left =   (glfwGetMouseButton(v->window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(v->window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(v->window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(v->window, &cursor_lastx, &cursor_lasty);
}

// mouse move
void MouseMove(double xpos, double ypos, slip_vis_t* v)
{
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - cursor_lastx;
    double dy = ypos - cursor_lasty;
    cursor_lastx = xpos;
    cursor_lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(v->window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(v->window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(v->window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    mjv_moveCamera(m, action, dx/height, dy/height, &v->scn, &v->cam);
}

void Scroll(double xoffset, double yoffset, slip_vis_t* v)
{
    // scroll: emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &v->scn, &v->cam);
}

// keyboard
void Keyboard(int key, int scancode, int act, int mods, slip_vis_t* v)
{
    // do not act on release
    if( act==GLFW_RELEASE )
        return;

    bUserInput = true;
}

void vis_close(slip_vis_t *v)
{
    if (!v || !v->window)
        return;

    // Free mujoco objects
    mjv_freeScene(&v->scn);
    mjr_freeContext(&v->con);

    // Close window
    glfwDestroyWindow(v->window);
    v->window = NULL;
}
