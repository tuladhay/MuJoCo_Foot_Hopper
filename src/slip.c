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
This program is meant to implement trajectory optimization for a SLIP with foot.
*/

static bool m_bMJActivated = false;
static bool glfw_initialized = false;

bool flag = false;

// See the XML file for the DOF/joint details
#define rootx 0
#define rootz 1
#define rot 2
#define leg_tau 3		// actuator here
#define leg_motor 4		// actuator here
#define spring 5		// toe spring
#define foot_joint 6	// actuator here

#define compression 1
#define thrust 2
#define flight 3

#define input_leg_tau 0
#define input_leg_motor 1
#define input_foot_joint 2

#define spring_stiffness 25000


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
		const char xmlfilename[] = "slip_with_foot_trajOpt.xml";
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
    return s;
}

void forward(slip_t* s, state_t* state)
{	//  Forward dynamics: same as mj_step but do not integrate in time. 

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

/******************************************************************************************
*******************************************************************************************
										ADVANCE STEP
*******************************************************************************************
*/
void step(slip_t* s, state_t* state)
{
	for (int i = 0; i < nU; i++)
		s->d->ctrl[i] = state->u[i];

	mj_step(m, s->d);		// pass the mjModel and mjData

	for (int i = 0; i < nQ; i++)
	{
		state->q[i] = s->d->qpos[i];
		state->qd[i] = s->d->qvel[i];
		state->qdd[i] = s->d->qacc[i];
	}
	state->t = s->d->time;
	state->cpos[0] = s->d->site_xpos[2];		// foot heel z,
	state->cpos[1] = s->d->site_xpos[5];		// foot toe z
	/*
	The size of xpos is nbody x 3. So the data corresponding to body n is:
	xpos[3*n], xpos[3*n+1], xpos[3*n+2].	- Emo Todorov
	*/
	get_CoP(s, state);
}

/******************************************************************************************
******************************************************************************************/



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










/***********************************************************************************
************************************************************************************
							ALL OPTIMIZATION RELATED FUNCTIONS
************************************************************************************/

void get_EoM_fields(slip_t* s, state_t* state, EoM_fields* EoM)
{
	// This function will return the extract the necessary EoM fields and populate it
	// so that in the Matlab side, I can calculate qdd for the dynamic constraints
	mjtNum mass[nQ*nQ];
	mjtNum h[nQ];
	mjtNum J[6*nQ];		// 3*nQ for each contact point. For two contact points
	mjtNum Jdot_Qdot[3*nC];	// temporarily store acceleration for each site in temp_acc[6]
	
	mju_zero(mass, nQ*nQ);
	mju_zero(h, nQ);
	mju_zero(J, 6*nQ);
	mju_zero(Jdot_Qdot, 3*nC);
	mjtNum tempJ[3*nQ];
	mju_zero(tempJ, 3*nQ);
	mjtNum temp_cacc[6];		// contact acceleration, J*qdd + Jdot*qdot = xdd, set qdd = 0
	mju_zero(temp_cacc, 6);

	//setting other zeros....
	mju_zero(s->d->qacc, nQ);
	mju_zero(s->d->xfrc_applied, m->nbody*6);
	mju_zero(s->d->qacc, m->nv);
	mju_zero(s->d->qacc_warmstart, m->nv);
	mju_zero(s->d->qfrc_applied, m->nv);

	//wipe out data and replace with state info
	for (int i = 0; i < nQ; i++)
	{
		s->d->qpos[i] = state->q[i];
		s->d->qvel[i] = state->qd[i];
	}

	for (int i = 0; i < nU; i++)
		s->d->ctrl[i] = state->u[i];

	mj_forward(m, s->d);
	// calculates the forward dynamics
	// this will not forward time, and update only vel and acc, not pos
	// mj_step() calculates dynamics and forwards in time

	// Get mass matrix (actually in array)
	mj_fullM(m, mass, s->d->qM);
	// d->qM is the sparse mass matrix
	for (int i = 0; i < (nQ*nQ); i++)
	{	// Copy into EoM struct
		EoM->H[i] = mass[i];
	}

	for (int i = 0; i < nQ; i++)
	{	// Extract coriolis, centripetal, gravity, spring terms
		h[i] = s->d->qfrc_bias[i] - s->d->qfrc_passive[i];
	}
	// Copy h to EoM struct
	for (int i = 0; i < nQ; i++)
	{
		EoM->h[i] = h[i];
	}

	// Extract J and Jdot_Qdot
	for (int i = 0; i < nC; i++)
	{	// Get the Jacobian for contact site[i]
		mj_jacSite(m, s->d, tempJ, NULL, i); // 3*nQ

		for (int j = 0; j < 3*nQ; j++)
		{	// stack both contact jacobians into one J array
			J[j + i*3*nQ] = tempJ[j];
		}

		// Get the contact site accelerations
		// Compute object 6D acceleration in object-centered frame, world/local orientation. 
		mj_objectAcceleration(m, s->d, mjOBJ_SITE, i, temp_cacc, 0);
		for (int j = 0; j < 3; j++)
		{
			Jdot_Qdot[j + i*3] = temp_cacc[j+3];
		}
	}
	// Copy J into EoM
	for (int i = 0; i < 6*nQ; i++)
	{	// Copying J into EoM struct
		EoM->J[i] = J[i];
	}

	// Copy Jdot_Qdot into EoM
	for (int i = 0; i < 3*nC; i++)
	{
		EoM->Jdot_Qdot[i] = Jdot_Qdot[i];
	}


}// get_EoM_fields










/***********************************************************************************
************************************************************************************
					RAIBERT STYLE CONTROLLER FOR FOOT PLACEMENT
************************************************************************************
*/
void controller(slip_t* s, state_t* state)
{
	/*****************************************************************************************************
					CHECK THESE CONFIGURATIONS, ESPECIALLY TOE BIAS, ANKLE ACTUATION, OPTION (for scaling)
	******************************************************************************************************
	*/
	double des_velocity = 2;			// m/s
	double gain_Kp_L0 = 6000;
	double gain_Kd_L0 = 200;
	double gain_Kp_swing = 2500;
	double gain_Kd_swing = 150;
	double gain_footDisp = 0.05;		// 0.14 worked best for point foot, and foot w/o ankle torque
	double gain_Kp_foot = 2000;
	double gain_Kd_foot = 150;

	double L_flight = 0.45;
	double L_extension = 0.55;			// 0.55 worked well for fixed flight time
	double xf_Point = 0;
	double rest_leg_length = 1.025;		// Add up the lengths in XML
	
	static double rec_time = 0;
	bool toe_bias = false;				// Set to True for lift off from toe
	double ankle_tau_d = 0;
	double CoP_pos = 0.1;				// Center of Pressure location where you want it to be
	int ankle_actuation_option = 1;		// see options below
	// 0: No ankle torque
	// 1: Torque scaling*
	// 2: Angle scaling* (*wrt previous stance time)
	// 3: CoP shift

/********************************************************************************************************
*********************************************************************************************************
*/
	// 1 = compression
	// 2 = thrust
	// 3 = flight

	// Check for the current dynamic state
	// Then use switch case to implement controller depending
	// on the dynamic state
	switch(state->dynamic_state)
	{
		case compression :
			// COMPRESSION to Thrust
			// check if body acceleration is greater than 0.0005
			// if body qdd > 0.005, dyn_state = 2, i.e THRUST
			// we can also try to do the same with:
			// Transition to thrust if leg (spring) begins to extend
			// if (state->qd[1] > 0.0)	// body z velocity
			if (state->qd[spring] < 0.0)		// spring relaxing
			{
				state->dynamic_state = thrust;
				rec_time = s->d->time;			// for ankle torque time scaling
				break;
			}
			break;

		case thrust :
			// THRUST to Flight
			// If foot contacts are above the ground, then dyn_state = FLIGHT
			if (  (state->cpos[0] > 0.01)   &&   (state->cpos[1] > 0.01))
			{
				state->dynamic_state = flight;
				state->stance_time = s->d->time - state->touchdown_time;

				// Calculate the desired touchdown angle depending on whether you want to account for toe bias
				// Adding bias reduces velocity tracking error. I think this is because due to the added foot, the toe touched the ground first, however,
				// the Raibert Controller assumes that it is the mid of the foot (point feet)
				if (toe_bias)
				{
					xf_Point = (0.5*(state->qd[rootx])*state->stance_time) + gain_footDisp*(state->apex_velocity - des_velocity)  -  0.01;		// Toe touchdown bias
				}
				else
				{
					xf_Point = (0.5*(state->qd[rootx])*state->stance_time) + gain_footDisp*(state->apex_velocity - des_velocity);				// NO BIAS
				}
				state->des_td_angle = asin(xf_Point/rest_leg_length);

				if (state->des_td_angle > 0.20)		// approx 10 degrees
					state->des_td_angle = 0.20;
				if (state->des_td_angle < -0.20)
					state->des_td_angle = -0.20;

				flag = true;	// flag to update the apex velocity
				rec_time = 0;
				break;
			}
			break;

		case flight :
			// FLIGHT to Compression
			// check if body acceleration is some negative threshold,
			// and there is ground contact then set state to compression

			if (state->qd[rootz] < 0 && flag)	// when it just crosses apex
			{
				state->apex_velocity = state->qd[rootx];	// horizontal velocity
				flag = false;
			}
			if (  ((state->cpos[0] < 0) || (state->cpos[1] < 0))   )
			{
				state->dynamic_state = compression;

				// Update the flight time
				state->touchdown_time = s->d->time;
				break;
			}
			break;
	}

	// ***************************************************************
	// Implement Force and Torque Controllers based on current state
	// u[0] is teg_tau
	// u[1] is leg_motor
	// u[2] is foot_joint

	switch(state->dynamic_state)
	{	/**********************************************************
		NOTE: CHECK THE TOE BIAS, ANGLE ACTUATION AND OTHER PARAMS
		**********************************************************/
		case 1 :	// Compression
		state->u[input_leg_tau] = 0;
		state->u[input_leg_motor] = (-gain_Kp_L0*(state->q[leg_motor] - L_flight) - gain_Kd_L0*state->qd[leg_motor]);
		state->u[input_foot_joint] = 0;
		break;

		case 2 :	// Thrust
		state->u[input_leg_tau] = 0;//-75;		// Trying some contant torque stance leg retraction
		state->u[input_leg_motor] = -gain_Kp_L0*(state->q[leg_motor]- L_extension) - gain_Kd_L0*state->qd[leg_motor];
		
		if (state->q[leg_motor] > 0.45)		// At what point during Thrust phase to apply ankle torque? This check might not be necessary
		{
				switch (ankle_actuation_option)
				{	
					case 0:
					{
						state->u[input_foot_joint] = 0;
					}
					break;

					case 1:		// scaling desired ankle torque by later half of stance time
					{			// need braces here because I am defining a variable inside case
						double target_torque = 100;		// Not guaranteed to reach this. Since stance time is estimate using old stance time
						state->u[input_foot_joint]	= ((s->d->time - rec_time)/(state->stance_time/2)) * target_torque;
					}
					break; 		// for time scaling torque
					
					case 2:		// scaling desired ankle angle by stance time, and using PD over the desired ankle angle
					{
						ankle_tau_d = ((s->d->time - rec_time)/(state->stance_time/2)) * 0.2;	// 0.2 is radians for max angle, it is approximate
						state->u[input_foot_joint] = -2000*(state->q[foot_joint] - ankle_tau_d) - 200*state->qd[foot_joint];
					}
					break;

					case 3:		// Apply torque to change the Center of Pressure to where you want it
					{	
						state->u[input_foot_joint] = ankle_tau_COP(state, CoP_pos);
					}
					break;// Break ankle actuation Switch
				}//switch
		}//if
		break;//Break thrust switch

		case 3 :	// Flight
		state->u[input_leg_tau] = -gain_Kp_swing*(state->q[leg_tau] - state->des_td_angle) - gain_Kd_swing*state->qd[leg_tau];	// Swing leg
		state->u[input_leg_motor] = -gain_Kp_L0*(state->q[leg_motor] - L_flight) - gain_Kd_L0*state->qd[leg_motor];
		state->u[input_foot_joint] = -gain_Kp_foot*(state->q[foot_joint] -  state->q[leg_tau]) - gain_Kd_foot*state->qd[foot_joint];
		break;
	}

	// Setting the control values
	for (int i = 0; i < nU; i++)
		{s->d->ctrl[i] = state->u[i];}

}

/************************************************************************************
*************************************************************************************
							CALCULATE CENTER OF PRESSURE
*************************************************************************************
*/
void get_CoP(slip_t* s, state_t* state)
{
	//*********************** METHOD 1, TAYLOR'S METHOD *****************************
	mjtNum conForce[s->d->ncon*6];
    mjtNum conJac[m->nv*(s->d->ncon*6)];
    mjtNum zForce[s->d->ncon];

    int idx = 0;
    for (int i = 0; i<s->d->nefc; i++)
    {   
        if (s->d->efc_type[i] != mjCNSTR_CONTACT_PYRAMIDAL)
            continue;
        conForce[idx] = s->d->efc_force[i];
                    
        for (int j = 0; j < m->nv; j++)
        {
            conJac[idx*m->nv + j] = s->d->efc_J[i*m->nv + j];
        }
        idx++;
    }// Till here we have grabbed the constrained Forces and the constrained Jacobian in arrays

/*
    for (int i = 0; i < s->d->ncon; i++)
    {
        mjtNum tempJac[m->nv*6];        // for a single contact
        mjtNum tempConForce[6];         // for a single contact

        for (int j = 0; j < 6; j++)
        {
            tempConForce[j] = conForce[i*6 + j];
            for (int k = 0; k < m->nv; k++)
            {
                tempJac[j*m->nv + k] = conJac[i*(6*m->nv) + j*m->nv + k];
            }
        }
        mjtNum resForce[m->nv];
        mju_mulMatTVec(resForce, tempJac, tempConForce, m->nv, 6);
        // printf("Contact Force: %d X, Z:(%f,%f)\t \t", i, resForce[0], resForce[1]);
        //printf("Contact Force: %d Z: %f\t ", i, resForce[1]);
        zForce[i] = resForce[1];	// getting the Z forces, index[0] are the X forces, according to the XML
    }

    mjtNum CoP = 0;
	if (s->d->ncon == 2)
	{
	    CoP = (zForce[0]*s->d->contact[0].pos[0] + zForce[1]*s->d->contact[1].pos[0]) / (zForce[0] + zForce[1]);
	    CoP = CoP - s->d->site_xpos[0];   // substract the site[0] position, as an approximation. Do trig later for actual position.
	}
	if (s->d->ncon == 1)
	{
	    CoP = s->d->contact[0].pos[0] - s->d->site_xpos[0] - 0.5854/100; //there was some bias
	} 
	// NOTE that now, the RANGE FOR CPOS is 0 to 20 (centimeters), with 10 being the center of the foot
	state->CoP = CoP;
*/



	//*********************************************************************************
	//********************** METHOD 2, USING INBUILT FUNCTION CALL ********************

	mjtNum contactForce1[6];    // Extract 6D force:torque for one contact, in contact frame.
	mjtNum contactForce2[6];
	mj_contactForce(m, s->d, 0, contactForce1);
	mj_contactForce(m, s->d, 1, contactForce2);
	    //printf("Contact force 1: %f\t", contactForce1[0]);  // [0] happens to be be the Z axis forces
	    //printf("Contact force 2: %f\n", contactForce2[0]);

	// GETTING CONTACT POSITIONS
	//printf("Contacts : %d\n", d->ncon);
	//printf("Contact Pos 1 : %f \t", s->d->contact[0].pos[0]);  // pos[0] is the X axis
	//printf("Contact Pos 2 : %f \t\n", s->d->contact[1].pos[0]);

	// CALCULATING CENTER OF PRESSURE
	mjtNum CoP = 0;
	if (s->d->ncon == 2)
	{
	    CoP = (contactForce1[0]*s->d->contact[0].pos[0] + contactForce2[0]*s->d->contact[1].pos[0]) / (contactForce1[0] + contactForce2[0]);
	    CoP = CoP - s->d->site_xpos[0];   // substract the site[0] position, as an approximation. Do trig later for actual position.
	}
	if (s->d->ncon == 1)
	{
	    // CoP = s->d->contact[0].pos[0] - s->d->site_xpos[0]; //there was some bias
	     //CoP = -1*s->d->contact[0].pos[0];
	    //CoP = -1*s->d->site_xpos[0];
	    CoP = (-1*s->d->contact[0].pos[0] + s->d->site_xpos[0])*10 +0.1;
	} 
	// NOTE that now, the RANGE FOR CPOS is 0 to 20 (centimeters), with 10 being the center of the foot
	state->CoP = CoP;

	//-------------------------------- For Saving to CSV -----------------------------------
	//--------------------------------------------------------------------------------------
    FILE *fp;

    double CoP_write;
    CoP_write = CoP;
    char *filename="CoP_output.csv";
     
    fp=fopen(filename,"a");
     
    //fprintf(fp,"CoP");
    fprintf(fp,"\n%f", CoP_write);
    fclose(fp);


}//get_CoP



/************************************************************************************
*************************************************************************************
							ANKLE TORQUE CONTROLLERS
*************************************************************************************
*/
double ankle_tau_COP(state_t* state, double CoP_pos)
{
	double force = spring_stiffness * state->q[spring];			// F = spring_stiffness * spring_displacement
	double torque = force * cos(state->q[leg_tau]) * CoP_pos;
	return torque;
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
	v->opt.flags[10] = true;	// to VISUALIZE THE CONTACT POINTS
	v->opt.flags[11] = true;	// to VISUALIZE THE CONTACT FORCES

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
