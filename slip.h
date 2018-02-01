#ifndef slip_h__
#define slip_h__
 

#include <stdbool.h>

#define nQ 7	// count the number of joints in the xml file
#define nU 3	// count the number of inputs in the xml file
#define nC 2	// count the number of sites in the xml file


typedef struct state_t {
	double q[nQ];
	double qd[nQ];
	double qdd[nQ];
	double u[nU];
	double t;
	double cpos[nC];
	int dynamic_state;
	double des_td_angle;
	double touchdown_time;
	double stance_time;
	double apex_velocity;
	double CoP;
} state_t;

typedef struct EoM_fields
{
	double H[nQ*nQ];		// Mass matrix
	double h[nQ];			// Coriolis, centripetal, gravity, spring
	double J[6*nQ];			// Jacobian to contact point
	double Jdot_Qdot[3*nC];		// Jdot_Qdot, contact accelerations, J*qdd + Jdot*qdot = xdd, set qdd = 0
	double qacc[nQ];
} EoM_fields;


typedef struct pos_limits_t {
	double lb[nQ];
	double ub[nQ];
} pos_limits_t;

typedef struct motor_limits_t {
	double lb[nU];
	double ub[nU];
} motor_limits_t;

typedef struct slip_t slip_t;
typedef struct slip_vis_t slip_vis_t;


slip_t* init(const char *basedir);
void forward(slip_t* s, state_t* state);
void step(slip_t* s, state_t* state);
void set_state(slip_t* s, state_t* state);
void run_forward(slip_t* s, state_t* state, double DT);
void step_ctrl(slip_t* s);
//int get_stationary_init(slip_t *s, state_t *state);
void get_joint_limits(pos_limits_t *lim);
void get_motor_limits(motor_limits_t *lim);

void controller(slip_t* s, state_t* state);
double ankle_tau_COP(state_t* state, double CoP_pos);
void get_CoP(slip_t* s, state_t* state);

void get_EoM_fields(slip_t* s, state_t* state, EoM_fields* EoM);

slip_vis_t *vis_init(void);
bool vis_draw(slip_vis_t *v, slip_t *s, bool bWaitUser);
void vis_close(slip_vis_t *v);
void Keyboard(int key, int scancode, int act, int mods, slip_vis_t* v);
void Scroll(double xoffset, double yoffset, slip_vis_t* v);
void MouseMove(double xpos, double ypos, slip_vis_t* v);
void MouseButton(int button, int act, int mods, slip_vis_t* v);

#endif  // slip_h__
