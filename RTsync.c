/*
 * This file has been reverse-engineered from RTsync.mexw64 and adapted
 * for non-Windows systems by Arne Hasselbring <arne.hasselbring@googlemail.com>
 * in January 2017.
 * Up to now this has been tested on Linux and macOS.
 * RTsync has originally been written by Sergey Shapovalov and released
 * on 2009-08-10.
 */

#ifdef _WIN32

#include <windows.h>

static double getTime()
{
    return (double)(clock()) / CLOCKS_PER_SEC;
}

#elif (defined __linux__)

#include <time.h>
#include <unistd.h>

static double getTime()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + (double)(ts.tv_nsec) / 1000000000;
}

#elif (defined __APPLE__)

#include <mach/mach_time.h>
#include <unistd.h>

static double getTime()
{
  static mach_timebase_info_data_t info = { 0, 0 };
  if (info.denom == 0) {
    mach_timebase_info(&info);
  }
  return (double)(mach_absolute_time() * (info.numer / info.denom)) / 1000000000;
}

#else
#error "Neither _WIN32 nor __linux__ nor __APPLE__ are defined."
#endif

#define S_FUNCTION_NAME  RTsync
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"


static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 2);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return;
    }

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 5);

    if (!ssSetNumInputPorts(S, 0)) return;
    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, 1);

    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);

    ssSetOptions(S, 0);
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    if (mxGetScalar(ssGetSFcnParam(S, 0)) == 2) {
        ssSetSampleTime(S, 0, mxGetScalar(ssGetSFcnParam(S, 1)));
        ssSetOffsetTime(S, 0, 0.0);
    } else {
        ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
        ssSetOffsetTime(S, 0, FIXED_IN_MINOR_STEP_OFFSET);
    }

}

#define MDL_START
static void mdlStart(SimStruct *S)
{
    double t = getTime();
    double t2 = ssGetT(S);
    ssGetDiscStates(S)[4] = 0;
    ssGetDiscStates(S)[1] = t2;
    ssGetDiscStates(S)[3] = t2;
    ssGetDiscStates(S)[0] = t;
    ssGetDiscStates(S)[2] = t;
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    *(double*)(ssGetOutputPortSignal(S, 0)) = ssGetDiscStates(S)[2] - ssGetDiscStates(S)[0] - ssGetDiscStates(S)[3];
}

#define MDL_UPDATE
static void mdlUpdate(SimStruct *S, int_T tid)
{
    double t, dt, t2, dt2, delta;
    t = getTime();
    dt = t - ssGetDiscStates(S)[2];
    ssGetDiscStates(S)[2] = t;
    t2 = ssGetT(S);
    if (dt >= 0) {
        dt2 = t2 - ssGetDiscStates(S)[3];
        ssGetDiscStates(S)[3] = t2;
        if (dt2 > 0) {
            delta = (t2 - ssGetDiscStates(S)[1] - (t - ssGetDiscStates(S)[0])) / dt2 * 0.1 + (dt2 - dt) * 0.5 + ssGetDiscStates(S)[4];
            ssGetDiscStates(S)[4] = delta;
            if (delta > 0) {
                dt2 *= 2;
                if (dt2 < delta) {
                    ssGetDiscStates(S)[4] = dt2;
                }
            } else {
                ssGetDiscStates(S)[4] = 0;
            }
        }
    } else {
        ssGetDiscStates(S)[0] = t;
        ssGetDiscStates(S)[1] = t2;
        ssGetDiscStates(S)[3] = t2;
    }
#ifdef _WIN32
    Sleep((DWORD)(ssGetDiscStates(S)[4] * 1000));
#else
    usleep((useconds_t)(ssGetDiscStates(S)[4] * 1000000));
#endif
}

static void mdlTerminate(SimStruct *S)
{
}


#ifdef  MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
