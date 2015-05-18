/*  File    : sfun_myo_v1.cpp
 *  Abstract:
 *
 *      Example of an C++ S-function which stores an C++ object in
 *      the pointers vector PWork.
 *
 *  Copyright 1990-2013 The MathWorks, Inc.
 */

#include <iostream>
#include <math.h>
#include <matrix.h>
#include <mex.h>

#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include "myo_class2a.hpp"

#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME  sfun_myo_v1

/*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 */
#include "simstruc.h"
#define U(element) (*uPtrs[element])  /* Pointer to Input Port0 */


#define IS_PARAM_DOUBLE(pVal) (mxIsNumeric(pVal) && !mxIsLogical(pVal) &&\
!mxIsEmpty(pVal) && !mxIsSparse(pVal) && !mxIsComplex(pVal) && mxIsDouble(pVal))

/*====================*
 * S-function methods *
 *====================*/

#define MDL_CHECK_PARAMETERS
#if defined(MDL_CHECK_PARAMETERS)  && defined(MATLAB_MEX_FILE)
/*
 * Check to make sure that each parameter is 1-d and positive
 */
static void mdlCheckParameters(SimStruct *S)
{

    const mxArray *pVal0 = ssGetSFcnParam(S,0);

    if ( !IS_PARAM_DOUBLE(pVal0)) {
        ssSetErrorStatus(S, "Parameter to S-function must be a double scalar");
        return;
    } 
}
#endif


/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 1);  /* Number of expected parameters */
#if defined(MATLAB_MEX_FILE)
    if (ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S)) {
        mdlCheckParameters(S);
        if (ssGetErrorStatus(S) != NULL) {
            return;
        }
    } else {
        return; /* Parameter mismatch will be reported by Simulink */
    }
#endif
    ssSetSFcnParamTunable(S, 0, 0);

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, 0)) return;
    //ssSetInputPortWidth(S, 0, 1);
    //ssSetInputPortDirectFeedThrough(S, 0, 1);
    
    if (!ssSetNumOutputPorts(S, 7)) return;
    ssSetOutputPortWidth(S, 0, 4);
    ssSetOutputPortWidth(S, 1, 8);
    ssSetOutputPortWidth(S, 2, 3);
    ssSetOutputPortWidth(S, 3, 3);
    ssSetOutputPortWidth(S, 4, 1);
    ssSetOutputPortWidth(S, 5, 1);
    ssSetOutputPortWidth(S, 6, 1);

    ssSetNumSampleTimes(S, PORT_BASED_SAMPLE_TIMES);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 3); // reserve element in the pointers vector
    ssSetNumModes(S, 0); // to store a C++ object
    ssSetNumNonsampledZCs(S, 0);

    ssSetSimStateCompliance(S, USE_CUSTOM_SIM_STATE);

    ssSetOptions(S, 0);
}

/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    This function is used to specify the sample time(s) for your
 *    S-function. You must register the same number of sample times as
 *    specified in ssSetNumSampleTimes.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, mxGetScalar(ssGetSFcnParam(S, 0)));
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
}

#define MDL_START  /* Change to #undef to remove function */
#if defined(MDL_START) 
  /* Function: mdlStart =======================================================
   * Abstract:
   *    This function is called once at start of model execution. If you
   *    have states that should be initialized once, this is the place
   *    to do it.
   */
  static void mdlStart(SimStruct *S)
  {         
      // store new C++ object in the      
      ssGetPWork(S)[0] = (void *) new myo::Hub;
      ssGetPWork(S)[1] = (void *) new DataCollector;
      ssGetPWork(S)[2] = (void *) new myo::Myo*;
      
      myo::Hub *hUb = (myo::Hub *)ssGetPWork(S)[0];
      myo::Myo *mYo = (myo::Myo *)ssGetPWork(S)[2];
      mYo = hUb->waitForMyo(10);
      
      // Enable EMG streaming
      mYo->unlock(myo::Myo::unlockHold);
      hUb->setLockingPolicy(hUb->lockingPolicyNone);
      mYo->setStreamEmg(myo::Myo::streamEmgEnabled);
      //mYo->vibrate(myo::Myo::vibrationLong);
      mYo->notifyUserAction();
   }                                            // pointers vector
  
#endif /*  MDL_START */
  
/* Function: mdlOutputs =======================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{   
    DataCollector *dataCollector = (DataCollector *) ssGetPWork(S)[1];
    myo::Hub *hUb = (myo::Hub *)ssGetPWork(S)[0];
    hUb->addListener(dataCollector);
    
    // In each iteration of our main loop, we run the Myo event loop for a set number of milliseconds.
    // In this case, we wish to update our display 20 times a second, so we run for 1000/20 milliseconds.
    hUb->run(1000/100);
    
    // retrieve C++ object from
    real_T  *y = ssGetOutputPortRealSignal(S,0); // the pointers vector and use
       
    *(y) = dataCollector->quat_x;
    *(y+1) = dataCollector->quat_y;
    *(y+2) = dataCollector->quat_z;
    *(y+3) = dataCollector->quat_w;
    
    *(y+4) = dataCollector->emgData[0];
    *(y+5) = dataCollector->emgData[1];
    *(y+6) = dataCollector->emgData[2];
    *(y+7) = dataCollector->emgData[3];
    *(y+8) = dataCollector->emgData[4];
    *(y+9) = dataCollector->emgData[5];
    *(y+10) = dataCollector->emgData[6];
    *(y+11) = dataCollector->emgData[7];
    
    *(y+12) = dataCollector->gyroData[0];
    *(y+13) = dataCollector->gyroData[1];
    *(y+14) = dataCollector->gyroData[2];
    
    *(y+15) = dataCollector->accData[0];
    *(y+16) = dataCollector->accData[1];
    *(y+17) = dataCollector->accData[2];    
    
    std::string currentPose = dataCollector->poseString;
     
    if (currentPose.compare("rest")==0){
        *(y+18) = 0; 
    }else if (currentPose.compare("fist")==0){
        *(y+18) = 1; 
    }else if (currentPose.compare("waveIn")==0){
        *(y+18) = 2;
    }else if (currentPose.compare("waveOut")==0){
        *(y+18) = 3;
    }else if (currentPose.compare("fingersSpread")==0){
        *(y+18) = 4;
    }else if (currentPose.compare("doubleTap")==0){
        *(y+18) = 5;
    }else if (currentPose.compare("unknown")==0){
        *(y+18) = 99;
    }
    
    if (dataCollector->onArm){
        *(y+19) = 1;
    }else{
        *(y+19) = 0;
    }
    
    if (dataCollector->swhichArm){
        *(y+20) = 1;
    }else{
        *(y+20) = 0;
    }
       
//     InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);
//     myo::Myo *mYO = (myo::Myo *)ssGetPWork(S)[3];
//     
//     if (*uPtrs[0]>0){
//         mYO->vibrate(myo::Myo::vibrationShort);
//     }
}    


/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
    // retrieve and destroy C++
    DataCollector *dataCollector = (DataCollector *) ssGetPWork(S)[1];
    myo::Hub *hUb = (myo::Hub *)ssGetPWork(S)[0];
    myo::Myo *mYo = (myo::Myo *)ssGetPWork(S)[2];
    myo::Myo *mYO = (myo::Myo *)ssGetPWork(S)[3];
    delete hUb;
    delete dataCollector;
    // object in the termination

}                                              

/*======================================================*
 * See sfuntmpl.doc for the optional S-function methods *
 *======================================================*/


/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif

