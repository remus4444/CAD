/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: controllerdemo.c
 *
 * Code generated for Simulink model 'controllerdemo'.
 *
 * Model version                  : 1.2
 * Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
 * C/C++ source code generated on : Mon Feb 16 19:57:05 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Atmel->AVR
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "controllerdemo.h"
#include "controllerdemo_private.h"
#include "rtwtypes.h"
#include <math.h>

/* Block states (default storage) */
DW_controllerdemo_T controllerdemo_DW;

/* Real-time model */
static RT_MODEL_controllerdemo_T controllerdemo_M_;
RT_MODEL_controllerdemo_T *const controllerdemo_M = &controllerdemo_M_;
real_T rt_roundd_snf(real_T u)
{
  real_T y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

/* Model step function */
void controllerdemo_step(void)
{
  real_T rtb_PulseGenerator;
  uint8_T tmp;

  /* DiscretePulseGenerator: '<Root>/Pulse Generator' */
  rtb_PulseGenerator = (controllerdemo_DW.clockTickCounter <
                        controllerdemo_P.PulseGenerator_Duty) &&
    (controllerdemo_DW.clockTickCounter >= 0L) ?
    controllerdemo_P.PulseGenerator_Amp : 0.0;
  if (controllerdemo_DW.clockTickCounter >=
      controllerdemo_P.PulseGenerator_Period - 1.0) {
    controllerdemo_DW.clockTickCounter = 0L;
  } else {
    controllerdemo_DW.clockTickCounter++;
  }

  /* End of DiscretePulseGenerator: '<Root>/Pulse Generator' */

  /* MATLABSystem: '<Root>/Digital Output' */
  rtb_PulseGenerator = rt_roundd_snf(rtb_PulseGenerator);
  if (rtb_PulseGenerator < 256.0) {
    if (rtb_PulseGenerator >= 0.0) {
      tmp = (uint8_T)rtb_PulseGenerator;
    } else {
      tmp = 0U;
    }
  } else {
    tmp = MAX_uint8_T;
  }

  writeDigitalPin(9, tmp);

  /* End of MATLABSystem: '<Root>/Digital Output' */
}

/* Model initialize function */
void controllerdemo_initialize(void)
{
  /* Start for MATLABSystem: '<Root>/Digital Output' */
  controllerdemo_DW.obj.matlabCodegenIsDeleted = false;
  controllerdemo_DW.obj.isInitialized = 1L;
  digitalIOSetup(9, 1);
  controllerdemo_DW.obj.isSetupComplete = true;
}

/* Model terminate function */
void controllerdemo_terminate(void)
{
  /* Terminate for MATLABSystem: '<Root>/Digital Output' */
  if (!controllerdemo_DW.obj.matlabCodegenIsDeleted) {
    controllerdemo_DW.obj.matlabCodegenIsDeleted = true;
  }

  /* End of Terminate for MATLABSystem: '<Root>/Digital Output' */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
