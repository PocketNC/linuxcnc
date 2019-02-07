/********************************************************************
* Description: switchKins.c
*   general switchKins for 3 axis Cartesian machine
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* License: GPL Version 2
*    
* Copyright (c) 2009 All rights reserved.
*
********************************************************************/

#include "motion.h"
#include "hal.h"
#include "rtapi.h"
#include "rtapi.h"		/* RTAPI realtime OS API */
#include "rtapi_app.h"		/* RTAPI realtime module decls */
#include "rtapi_math.h"
#include "rtapi_string.h"

// sequential joint number assignments
#define JX 0
#define JY 1
#define JZ 2

#define JA 3
#define JC 4

struct haldata {
    hal_float_t *z_offset;
    hal_float_t *y_offset;
    hal_float_t *tool_offset;
} *haldata;

struct data { 
    hal_s32_t joints[EMCMOT_MAX_JOINTS];
    hal_float_t kinsSwitchVar0;
    hal_float_t kinsSwitchVar1;
} *data;

#define SET(f) pos->f = (joints[i])

int kinematicsSwitch(double adjustKinsVar0, double adjustKinsVar1)
{
    data->kinsSwitchVar0 = adjustKinsVar0;
    data->kinsSwitchVar1 = adjustKinsVar1;
    if(data->kinsSwitchVar0 == 1){
        rtapi_print_msg(RTAPI_MSG_ERR, "switchKins: INFO: kinematicsSwitch <AC>\n");
    }
    else{
        rtapi_print_msg(RTAPI_MSG_ERR, "switchKins: INFO: kinematicsSwitch <Triv>\n");
    }

    return 1;
}

static 
int trivKinematicsForward(const double *joints,
              EmcPose * pos,
              const KINEMATICS_FORWARD_FLAGS * fflags,
              KINEMATICS_INVERSE_FLAGS * iflags)
{
    int i;

    for(i = 0; i < EMCMOT_MAX_JOINTS; i++) {
        switch(data->joints[i]) {
            case 0: SET(tran.x); break;
            case 1: SET(tran.y); break;
            case 2: SET(tran.z); break;
            case 3: SET(a); break;
            case 4: SET(c); break;
            case 5: SET(b); break;
            case 6: SET(u); break;
            case 7: SET(v); break;
            case 8: SET(w); break;
        }
    }

    return 0;
}

static 
int xyzabKinematicsForward(const double *joints,
              EmcPose * pos,
              const KINEMATICS_FORWARD_FLAGS * fflags,
              KINEMATICS_INVERSE_FLAGS * iflags)
{
    double    dy = *(haldata->y_offset);
    double    dz = *(haldata->z_offset);
    double    dt = *(haldata->tool_offset);
    double a_rad = joints[JA]*TO_RAD;
    double c_rad = joints[JC]*TO_RAD;

    dz = dz + dt;

    pos->tran.x = + cos(c_rad)              * (joints[JX]     )
                  + sin(c_rad) * cos(a_rad) * (joints[JY] - dy)
                  + sin(c_rad) * sin(a_rad) * (joints[JZ] - dz)
                  + sin(c_rad) * dy;

    pos->tran.y = - sin(c_rad)              * (joints[JX]     )
                  + cos(c_rad) * cos(a_rad) * (joints[JY] - dy)
                  + cos(c_rad) * sin(a_rad) * (joints[JZ] - dz)
                  + cos(c_rad) * dy;

    pos->tran.z = + 0
                  - sin(a_rad) * (joints[JY] - dy)
                  + cos(a_rad) * (joints[JZ] - dz)
                  + dz;

    pos->a = joints[JA];
    pos->c = joints[JC];

    pos->b = 0;
    pos->w = 0;
    pos->u = 0;
    pos->v = 0;

    return 0;
}

int kinematicsForward(const double *joints,
		      EmcPose * pos,
		      const KINEMATICS_FORWARD_FLAGS * fflags,
		      KINEMATICS_INVERSE_FLAGS * iflags)
{
    if(data->kinsSwitchVar0 == 1){
        return xyzabKinematicsForward( joints, pos, fflags, iflags );
    }
    else{
        return trivKinematicsForward( joints, pos, fflags, iflags );
    }

    return 0;
}

static
int xyzabKinematicsInverse(const EmcPose * pos,
              double *joints,
              const KINEMATICS_INVERSE_FLAGS * iflags,
              KINEMATICS_FORWARD_FLAGS * fflags)
{
    double    dz = *(haldata->z_offset);
    double    dy = *(haldata->y_offset);
    double    dt = *(haldata->tool_offset);
    double c_rad = pos->c*TO_RAD;
    double a_rad = pos->a*TO_RAD;

    dz = dz + dt;

    joints[JX] = + cos(c_rad) * pos->tran.x
                 - sin(c_rad) * pos->tran.y;

    joints[JY] = + sin(c_rad) * cos(a_rad) * pos->tran.x
                 + cos(c_rad) * cos(a_rad) * pos->tran.y
                 - sin(a_rad)              * pos->tran.z
                 - cos(a_rad) * dy
                 + sin(a_rad) * dz + dy;

    joints[JZ] = + sin(c_rad) * sin(a_rad) * pos->tran.x
                 + cos(c_rad) * sin(a_rad) * pos->tran.y
                 + cos(a_rad)              * pos->tran.z
                 - sin(a_rad) * dy
                 - cos(a_rad) * dz
                 + dz;

    joints[JA] = pos->a;
    joints[JC] = pos->c;

    return 0;
}

static
int trivKinematicsInverse(const EmcPose * pos,
              double *joints,
              const KINEMATICS_INVERSE_FLAGS * iflags,
              KINEMATICS_FORWARD_FLAGS * fflags)
{
    int i;
    for(i = 0; i < EMCMOT_MAX_JOINTS; i++) {
        switch(data->joints[i]) {
            case 0: joints[i] = pos->tran.x; break;
            case 1: joints[i] = pos->tran.y; break;
            case 2: joints[i] = pos->tran.z; break;
            case 3: joints[i] = pos->a; break;
            case 4: joints[i] = pos->c; break;
            case 5: joints[i] = pos->b; break;
            case 6: joints[i] = pos->u; break;
            case 7: joints[i] = pos->v; break;
            case 8: joints[i] = pos->w; break;
        }
    }

    return 0;
}

int kinematicsInverse(const EmcPose * pos,
		      double *joints,
		      const KINEMATICS_INVERSE_FLAGS * iflags,
		      KINEMATICS_FORWARD_FLAGS * fflags)
{
    if(data->kinsSwitchVar0 == 1){
        return xyzabKinematicsInverse( pos, joints, iflags, fflags );
    }
    else{
        return trivKinematicsInverse( pos, joints, iflags, fflags );
    }

    return 0;
}

/* implemented for these kinematics as giving joints preference */
int kinematicsHome(EmcPose * world,
		   double *joint,
		   KINEMATICS_FORWARD_FLAGS * fflags,
		   KINEMATICS_INVERSE_FLAGS * iflags)
{
    *fflags = 0;
    *iflags = 0;

    return kinematicsForward(joint, world, fflags, iflags);
}

static KINEMATICS_TYPE ktype = -1;

KINEMATICS_TYPE kinematicsType()
{
    return ktype;
}

static char *coordinates = "XYZABCUVW";
RTAPI_MP_STRING(coordinates, "Existing Axes");

static char *kinstype = "1"; // use KINEMATICS_IDENTITY
RTAPI_MP_STRING(kinstype, "Kinematics Type (Identity,Both)");

EXPORT_SYMBOL(kinematicsType);
EXPORT_SYMBOL(kinematicsForward);
EXPORT_SYMBOL(kinematicsSwitch);
EXPORT_SYMBOL(kinematicsInverse);
MODULE_LICENSE("GPL");

static int next_axis_number(void) {
    while(*coordinates) {
	switch(*coordinates) {
	    case 'x': case 'X': coordinates++; return 0;
	    case 'y': case 'Y': coordinates++; return 1;
	    case 'z': case 'Z': coordinates++; return 2;
	    case 'a': case 'A': coordinates++; return 3;
	    case 'b': case 'B': coordinates++; return 4;
	    case 'c': case 'C': coordinates++; return 5;
	    case 'u': case 'U': coordinates++; return 6;
	    case 'v': case 'V': coordinates++; return 7;
	    case 'w': case 'W': coordinates++; return 8;
	    case ' ': case '\t': coordinates++; continue;
	}
	rtapi_print_msg(RTAPI_MSG_ERR,
		"switchKins: ERROR: Invalid character '%c' in coordinates\n",
		*coordinates);
		return -1;
    }
    return -1;
}

int comp_id;
int rtapi_app_main(void) {
    int i;
    int res = 0;
    comp_id = hal_init("switchKins");
    if(comp_id < 0) return comp_id;

    data = hal_malloc(sizeof(struct data));

    for(i=0; i<EMCMOT_MAX_JOINTS; i++) {
    	data->joints[i] = next_axis_number();
    }
    data->kinsSwitchVar0 = 0;
    switch (*kinstype) {
      case 'b': case 'B': ktype = KINEMATICS_BOTH;         break;
      case 'f': case 'F': ktype = KINEMATICS_FORWARD_ONLY; break;
      case 'i': case 'I': ktype = KINEMATICS_INVERSE_ONLY; break;
      case '1': default:  ktype = KINEMATICS_IDENTITY;
    }
    
    haldata = hal_malloc(sizeof(struct haldata));

    if((res = hal_pin_float_new("xyzac-trt-kins.y-offset",
              HAL_IO, &(haldata->y_offset), comp_id)) < 0) goto error;
    if((res = hal_pin_float_new("xyzac-trt-kins.z-offset",
              HAL_IO, &(haldata->z_offset), comp_id)) < 0) goto error;
    if((res = hal_pin_float_new("xyzac-trt-kins.tool-offset",
              HAL_IN, &(haldata->tool_offset), comp_id)) < 0) goto error;


    rtapi_print_msg(RTAPI_MSG_ERR, "switchKins: kinstype <%d>\n", ktype);
    hal_ready(comp_id);
    return 0;

error:
    hal_exit(comp_id);
    return res;
}

void rtapi_app_exit(void) { hal_exit(comp_id); }
