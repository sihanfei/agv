/*
 * File: PNC_StateFlow_20190412_2.c
 *
 * Code generated for Simulink model 'PNC_StateFlow_20190412_2'.
 *
 * Model version                  : 1.355
 * Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
 * C/C++ source code generated on : Fri Apr 12 13:17:15 2019
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "PNC_StateFlow_20190412_2.h"

/* Named constants for Chart: '<Root>/PNC' */
#define IN_Charge (( uint8_T )1U)
#define IN_ControlledStop (( uint8_T )1U)
#define IN_EmergencyStop (( uint8_T )2U)
#define IN_ExceptionHandling (( uint8_T )1U)
#define IN_Latched (( uint8_T )3U)
#define IN_Liftdown (( uint8_T )2U)
#define IN_Liftup (( uint8_T )3U)
#define IN_Manual (( uint8_T )2U)
#define IN_Moving (( uint8_T )4U)
#define IN_NO_ACTIVE_CHILD (( uint8_T )0U)
#define IN_NormalOperation (( uint8_T )3U)
#define IN_Standby (( uint8_T )4U)
#define IN_Standstill (( uint8_T )5U)
#define event_Break (14)
#define event_Bypass (17)
#define event_Charge_Job (8)
#define event_Charge_OK (9)
#define event_Estop (12)
#define event_Liftdown_Job (6)
#define event_Liftdown_OK (7)
#define event_Liftup_Job (4)
#define event_Liftup_OK (5)
#define event_Moving_Job (10)
#define event_Moving_OK (11)
#define event_Reset (16)
#define event_Runing_Break (15)
#define event_Runing_Estop (13)
#define event_Stop_ok (18)
#define event_Switch_Off_Manual (1)
#define event_Switch_On_Manual (0)
#define event_Task_Cmd_Received (2)
#define event_Task_Null (3)
#include "solver_zc.h"
#include "zero_crossing_types.h"
#ifndef slZcHadEvent
#define slZcHadEvent(ev, zcsDir) (((ev) & (zcsDir)) != 0x00)
#endif

#ifndef slZcUnAliasEvents
#define slZcUnAliasEvents(evL, evR)                                                                                    \
  ((((slZcHadEvent((evL), (SL_ZCS_EVENT_N2Z)) && slZcHadEvent((evR), (SL_ZCS_EVENT_Z2P))) ||                           \
     (slZcHadEvent((evL), (SL_ZCS_EVENT_P2Z)) && slZcHadEvent((evR), (SL_ZCS_EVENT_Z2N))))                             \
        ? (SL_ZCS_EVENT_NUL)                                                                                           \
        : (evR)))
#endif

/* Block signals and states (auto storage) */
DW rtDW;

/* Previous zero-crossings (trigger) states */
PrevZCX rtPrevZCX;

/* External inputs (root inport signals with auto storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with auto storage) */
ExtY rtY;

/* Real-time model */
RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;

/* Forward declaration for local functions */
static void chartstep_c2_PNC_StateFlow_2019(const int32_T *sfEvent);
extern ZCEventType rt_ZCFcn(ZCDirection zcDir, ZCSigState *prevZc, real_T currValue);

/* Detect zero crossings events. */
ZCEventType rt_ZCFcn(ZCDirection zcDir, ZCSigState *prevZc, real_T currValue)
{
  slZcEventType zcsDir;
  slZcEventType tempEv;
  ZCEventType zcEvent = NO_ZCEVENT; /* assume */

  /* zcEvent matrix */
  static const slZcEventType eventMatrix[4][4] = {
      /*          ZER              POS              NEG              UNK */
      {SL_ZCS_EVENT_NUL, SL_ZCS_EVENT_Z2P, SL_ZCS_EVENT_Z2N, SL_ZCS_EVENT_NUL}, /* ZER */

      {SL_ZCS_EVENT_P2Z, SL_ZCS_EVENT_NUL, SL_ZCS_EVENT_P2N, SL_ZCS_EVENT_NUL}, /* POS */

      {SL_ZCS_EVENT_N2Z, SL_ZCS_EVENT_N2P, SL_ZCS_EVENT_NUL, SL_ZCS_EVENT_NUL}, /* NEG */

      {SL_ZCS_EVENT_NUL, SL_ZCS_EVENT_NUL, SL_ZCS_EVENT_NUL, SL_ZCS_EVENT_NUL} /* UNK */
  };

  /* get prevZcEvent and prevZcSign from prevZc */
  slZcEventType prevEv        = (slZcEventType)(((uint8_T)(*prevZc)) >> 2);
  slZcSignalSignType prevSign = (slZcSignalSignType)(((uint8_T)(*prevZc)) & ( uint8_T )0x03);

  /* get current zcSignal sign from current zcSignal value */
  slZcSignalSignType currSign = (slZcSignalSignType)(
      (currValue) > 0.0 ? SL_ZCS_SIGN_POS : ((currValue) < 0.0 ? SL_ZCS_SIGN_NEG : SL_ZCS_SIGN_ZERO));

  /* get current zcEvent based on prev and current zcSignal value */
  slZcEventType currEv = eventMatrix[prevSign][currSign];

  /* get slZcEventType from ZCDirection */
  switch (zcDir)
  {
  case ANY_ZERO_CROSSING:
    zcsDir = SL_ZCS_EVENT_ALL;
    break;

  case FALLING_ZERO_CROSSING:
    zcsDir = SL_ZCS_EVENT_ALL_DN;
    break;

  case RISING_ZERO_CROSSING:
    zcsDir = SL_ZCS_EVENT_ALL_UP;
    break;

  default:
    zcsDir = SL_ZCS_EVENT_NUL;
    break;
  }

  /*had event, check if double zc happend remove double detection. */
  if (slZcHadEvent(currEv, zcsDir))
  {
    currEv = (slZcEventType)(slZcUnAliasEvents(prevEv, currEv));
  }
  else
  {
    currEv = SL_ZCS_EVENT_NUL;
  }

  /* Update prevZc */
  tempEv  = (slZcEventType)(currEv << 2); /* shift left by 2 bits */
  *prevZc = (ZCSigState)((currSign) | (tempEv));
  if ((currEv & SL_ZCS_EVENT_ALL_DN) != 0)
  {
    zcEvent = FALLING_ZCEVENT;
  }
  else if ((currEv & SL_ZCS_EVENT_ALL_UP) != 0)
  {
    zcEvent = RISING_ZCEVENT;
  }
  else
  {
    zcEvent = NO_ZCEVENT;
  }

  return zcEvent;
} /* rt_ZCFcn */

/* Function for Chart: '<Root>/PNC' */
static void chartstep_c2_PNC_StateFlow_2019(const int32_T *sfEvent)
{
  /* During: PNC */
  switch (rtDW.is_c2_PNC_StateFlow_20190412_2)
  {
  case IN_ExceptionHandling:
    /* During 'ExceptionHandling': '<S1>:51' */
    switch (rtDW.is_ExceptionHandling)
    {
    case IN_ControlledStop:
      /* During 'ControlledStop': '<S1>:52' */
      if (*sfEvent == event_Runing_Break)
      {
        /* Transition: '<S1>:57' */
        rtDW.is_ExceptionHandling = IN_Latched;

        /* Entry 'Latched': '<S1>:56' */
        rtY.PNC_SF_Status = 10;
      }
      break;

    case IN_EmergencyStop:
      /* During 'EmergencyStop': '<S1>:53' */
      if (*sfEvent == event_Runing_Estop)
      {
        /* Transition: '<S1>:58' */
        rtDW.is_ExceptionHandling = IN_Latched;

        /* Entry 'Latched': '<S1>:56' */
        rtY.PNC_SF_Status = 10;
      }
      break;

    default:
      /* During 'Latched': '<S1>:56' */
      switch (*sfEvent)
      {
      case event_Switch_On_Manual:
        /* Transition: '<S1>:59' */
        rtDW.is_ExceptionHandling           = IN_NO_ACTIVE_CHILD;
        rtDW.is_c2_PNC_StateFlow_20190412_2 = IN_Manual;

        /* Entry 'Manual': '<S1>:43' */
        rtY.PNC_SF_Status = 7;
        break;

      case event_Reset:
        /* Transition: '<S1>:63' */
        rtDW.is_ExceptionHandling           = IN_NO_ACTIVE_CHILD;
        rtDW.is_c2_PNC_StateFlow_20190412_2 = IN_NormalOperation;
        rtDW.is_NormalOperation             = IN_Standstill;

        /* Entry 'Standstill': '<S1>:17' */
        rtY.PNC_SF_Status = 2;
        break;

      case event_Bypass:
        /* Transition: '<S1>:64' */
        rtDW.is_ExceptionHandling           = IN_NO_ACTIVE_CHILD;
        rtDW.is_c2_PNC_StateFlow_20190412_2 = IN_NormalOperation;
        rtDW.is_NormalOperation             = IN_Standstill;

        /* Entry 'Standstill': '<S1>:17' */
        rtY.PNC_SF_Status = 2;
        break;
      }
      break;
    }
    break;

  case IN_Manual:
    /* During 'Manual': '<S1>:43' */
    if (*sfEvent == event_Switch_Off_Manual)
    {
      /* Transition: '<S1>:45' */
      rtDW.is_c2_PNC_StateFlow_20190412_2 = IN_Standby;

      /* Entry 'Standby': '<S1>:42' */
      rtY.PNC_SF_Status = 1;
    }
    break;

  case IN_NormalOperation:
    /* During 'NormalOperation': '<S1>:50' */
    switch (*sfEvent)
    {
    case event_Estop:
      /* Transition: '<S1>:55' */
      /* Exit Internal 'NormalOperation': '<S1>:50' */
      rtDW.is_NormalOperation             = IN_NO_ACTIVE_CHILD;
      rtDW.is_c2_PNC_StateFlow_20190412_2 = IN_ExceptionHandling;
      rtDW.is_ExceptionHandling           = IN_EmergencyStop;

      /* Entry 'EmergencyStop': '<S1>:53' */
      rtY.PNC_SF_Status = 9;
      break;

    case event_Break:
      /* Transition: '<S1>:54' */
      /* Exit Internal 'NormalOperation': '<S1>:50' */
      rtDW.is_NormalOperation             = IN_NO_ACTIVE_CHILD;
      rtDW.is_c2_PNC_StateFlow_20190412_2 = IN_ExceptionHandling;
      rtDW.is_ExceptionHandling           = IN_ControlledStop;

      /* Entry 'ControlledStop': '<S1>:52' */
      rtY.PNC_SF_Status = 8;
      break;

    default:
      switch (rtDW.is_NormalOperation)
      {
      case IN_Charge:
        /* During 'Charge': '<S1>:16' */
        if (*sfEvent == event_Charge_OK)
        {
          /* Transition: '<S1>:21' */
          rtDW.is_NormalOperation = IN_Standstill;

          /* Entry 'Standstill': '<S1>:17' */
          rtY.PNC_SF_Status = 2;
        }
        break;

      case IN_Liftdown:
        /* During 'Liftdown': '<S1>:22' */
        if (*sfEvent == event_Liftdown_OK)
        {
          /* Transition: '<S1>:24' */
          rtDW.is_NormalOperation = IN_Standstill;

          /* Entry 'Standstill': '<S1>:17' */
          rtY.PNC_SF_Status = 2;
        }
        break;

      case IN_Liftup:
        /* During 'Liftup': '<S1>:15' */
        if (*sfEvent == event_Liftup_OK)
        {
          /* Transition: '<S1>:19' */
          rtDW.is_NormalOperation = IN_Standstill;

          /* Entry 'Standstill': '<S1>:17' */
          rtY.PNC_SF_Status = 2;
        }
        break;

      case IN_Moving:
        /* During 'Moving': '<S1>:114' */
        if (*sfEvent == event_Moving_OK)
        {
          /* Transition: '<S1>:116' */
          rtDW.is_NormalOperation = IN_Standstill;

          /* Entry 'Standstill': '<S1>:17' */
          rtY.PNC_SF_Status = 2;
        }
        break;

      default:
        /* During 'Standstill': '<S1>:17' */
        switch (*sfEvent)
        {
        case event_Liftup_Job:
          /* Transition: '<S1>:18' */
          rtDW.is_NormalOperation = IN_Liftup;

          /* Entry 'Liftup': '<S1>:15' */
          rtY.PNC_SF_Status = 4;
          break;

        case event_Liftdown_Job:
          /* Transition: '<S1>:23' */
          rtDW.is_NormalOperation = IN_Liftdown;

          /* Entry 'Liftdown': '<S1>:22' */
          rtY.PNC_SF_Status = 5;
          break;

        case event_Charge_Job:
          /* Transition: '<S1>:20' */
          rtDW.is_NormalOperation = IN_Charge;

          /* Entry 'Charge': '<S1>:16' */
          rtY.PNC_SF_Status = 6;
          break;

        case event_Task_Null:
          /* Transition: '<S1>:49' */
          rtDW.is_NormalOperation             = IN_NO_ACTIVE_CHILD;
          rtDW.is_c2_PNC_StateFlow_20190412_2 = IN_Standby;

          /* Entry 'Standby': '<S1>:42' */
          rtY.PNC_SF_Status = 1;
          break;

        case event_Moving_Job:
          /* Transition: '<S1>:150' */
          rtDW.is_NormalOperation = IN_Moving;

          /* Entry 'Moving': '<S1>:114' */
          rtY.PNC_SF_Status = 3;
          break;
        }
        break;
      }
      break;
    }
    break;

  default:
    /* During 'Standby': '<S1>:42' */
    switch (*sfEvent)
    {
    case event_Switch_On_Manual:
      /* Transition: '<S1>:44' */
      rtDW.is_c2_PNC_StateFlow_20190412_2 = IN_Manual;

      /* Entry 'Manual': '<S1>:43' */
      rtY.PNC_SF_Status = 7;
      break;

    case event_Task_Cmd_Received:
      /* Transition: '<S1>:46' */
      rtDW.is_c2_PNC_StateFlow_20190412_2 = IN_NormalOperation;
      rtDW.is_NormalOperation             = IN_Standstill;

      /* Entry 'Standstill': '<S1>:17' */
      rtY.PNC_SF_Status = 2;
      break;
    }
    break;
  }
}

/* Model step function */
void PNC_StateFlow_20190412_2_step(void)
{
  ZCEventType zcEvent[19];
  int8_T rtb_inputevents[19];
  int32_T i;
  boolean_T tmp;

  /* Chart: '<Root>/PNC' incorporates:
   *  TriggerPort: '<S1>/input events'
   */
  /* Inport: '<Root>/Switch_On_Manual' */
  zcEvent[0] = rt_ZCFcn(RISING_ZERO_CROSSING, &rtPrevZCX.PNC_Trig_ZCE[0], (rtU.Switch_On_Manual));

  /* Inport: '<Root>/Switch_Off_Manual' */
  zcEvent[1] = rt_ZCFcn(RISING_ZERO_CROSSING, &rtPrevZCX.PNC_Trig_ZCE[1], (rtU.Switch_Off_Manual));

  /* Inport: '<Root>/Task_Cmd_Received' */
  zcEvent[2] = rt_ZCFcn(RISING_ZERO_CROSSING, &rtPrevZCX.PNC_Trig_ZCE[2], (rtU.Task_Cmd_Received));

  /* Inport: '<Root>/Task_Null' */
  zcEvent[3] = rt_ZCFcn(RISING_ZERO_CROSSING, &rtPrevZCX.PNC_Trig_ZCE[3], (rtU.Task_Null));

  /* Inport: '<Root>/Liftup_Job' */
  zcEvent[4] = rt_ZCFcn(RISING_ZERO_CROSSING, &rtPrevZCX.PNC_Trig_ZCE[4], (rtU.Liftup_Job));

  /* Inport: '<Root>/Liftup_OK' */
  zcEvent[5] = rt_ZCFcn(RISING_ZERO_CROSSING, &rtPrevZCX.PNC_Trig_ZCE[5], (rtU.Liftup_OK));

  /* Inport: '<Root>/Liftdown_Job' */
  zcEvent[6] = rt_ZCFcn(RISING_ZERO_CROSSING, &rtPrevZCX.PNC_Trig_ZCE[6], (rtU.Liftdown_Job));

  /* Inport: '<Root>/Liftdown_OK' */
  zcEvent[7] = rt_ZCFcn(RISING_ZERO_CROSSING, &rtPrevZCX.PNC_Trig_ZCE[7], (rtU.Liftdown_OK));

  /* Inport: '<Root>/Charge_Job' */
  zcEvent[8] = rt_ZCFcn(RISING_ZERO_CROSSING, &rtPrevZCX.PNC_Trig_ZCE[8], (rtU.Charge_Job));

  /* Inport: '<Root>/Charge_OK' */
  zcEvent[9] = rt_ZCFcn(RISING_ZERO_CROSSING, &rtPrevZCX.PNC_Trig_ZCE[9], (rtU.Charge_OK));

  /* Inport: '<Root>/Moving_Job' */
  zcEvent[10] = rt_ZCFcn(RISING_ZERO_CROSSING, &rtPrevZCX.PNC_Trig_ZCE[10], (rtU.Moving_Job));

  /* Inport: '<Root>/Moving_OK' */
  zcEvent[11] = rt_ZCFcn(RISING_ZERO_CROSSING, &rtPrevZCX.PNC_Trig_ZCE[11], (rtU.Moving_OK));

  /* Inport: '<Root>/Estop' */
  zcEvent[12] = rt_ZCFcn(RISING_ZERO_CROSSING, &rtPrevZCX.PNC_Trig_ZCE[12], (rtU.Estop));

  /* Inport: '<Root>/Runing_Estop' */
  zcEvent[13] = rt_ZCFcn(RISING_ZERO_CROSSING, &rtPrevZCX.PNC_Trig_ZCE[13], (rtU.Runing_Estop));

  /* Inport: '<Root>/Break' */
  zcEvent[14] = rt_ZCFcn(RISING_ZERO_CROSSING, &rtPrevZCX.PNC_Trig_ZCE[14], (rtU.Break));

  /* Inport: '<Root>/Runing_Break' */
  zcEvent[15] = rt_ZCFcn(RISING_ZERO_CROSSING, &rtPrevZCX.PNC_Trig_ZCE[15], (rtU.Runing_Break));

  /* Inport: '<Root>/Reset ' */
  zcEvent[16] = rt_ZCFcn(RISING_ZERO_CROSSING, &rtPrevZCX.PNC_Trig_ZCE[16], (rtU.Reset));

  /* Inport: '<Root>/Bypass ' */
  zcEvent[17] = rt_ZCFcn(RISING_ZERO_CROSSING, &rtPrevZCX.PNC_Trig_ZCE[17], (rtU.Bypass));

  /* Inport: '<Root>/Stop_ok ' */
  zcEvent[18] = rt_ZCFcn(RISING_ZERO_CROSSING, &rtPrevZCX.PNC_Trig_ZCE[18], (rtU.Stop_ok));
  tmp         = false;
  for (i = 0; i < 19; i++)
  {
    tmp = (tmp || (zcEvent[i] != NO_ZCEVENT));
  }

  if (tmp)
  {
    for (i = 0; i < 19; i++)
    {
      rtb_inputevents[i] = ( int8_T )zcEvent[i];
    }

    /* Gateway: PNC */
    if (rtb_inputevents[0U] == 1)
    {
      /* Event: '<S1>:133' */
      i = event_Switch_On_Manual;
      chartstep_c2_PNC_StateFlow_2019(&i);
    }

    if (rtb_inputevents[1U] == 1)
    {
      /* Event: '<S1>:134' */
      i = event_Switch_Off_Manual;
      chartstep_c2_PNC_StateFlow_2019(&i);
    }

    if (rtb_inputevents[2U] == 1)
    {
      /* Event: '<S1>:119' */
      i = event_Task_Cmd_Received;
      chartstep_c2_PNC_StateFlow_2019(&i);
    }

    if (rtb_inputevents[3U] == 1)
    {
      /* Event: '<S1>:120' */
      i = event_Task_Null;
      chartstep_c2_PNC_StateFlow_2019(&i);
    }

    if (rtb_inputevents[4U] == 1)
    {
      /* Event: '<S1>:121' */
      i = event_Liftup_Job;
      chartstep_c2_PNC_StateFlow_2019(&i);
    }

    if (rtb_inputevents[5U] == 1)
    {
      /* Event: '<S1>:122' */
      i = event_Liftup_OK;
      chartstep_c2_PNC_StateFlow_2019(&i);
    }

    if (rtb_inputevents[6U] == 1)
    {
      /* Event: '<S1>:123' */
      i = event_Liftdown_Job;
      chartstep_c2_PNC_StateFlow_2019(&i);
    }

    if (rtb_inputevents[7U] == 1)
    {
      /* Event: '<S1>:124' */
      i = event_Liftdown_OK;
      chartstep_c2_PNC_StateFlow_2019(&i);
    }

    if (rtb_inputevents[8U] == 1)
    {
      /* Event: '<S1>:125' */
      i = event_Charge_Job;
      chartstep_c2_PNC_StateFlow_2019(&i);
    }

    if (rtb_inputevents[9U] == 1)
    {
      /* Event: '<S1>:126' */
      i = event_Charge_OK;
      chartstep_c2_PNC_StateFlow_2019(&i);
    }

    if (rtb_inputevents[10U] == 1)
    {
      /* Event: '<S1>:127' */
      i = event_Moving_Job;
      chartstep_c2_PNC_StateFlow_2019(&i);
    }

    if (rtb_inputevents[11U] == 1)
    {
      /* Event: '<S1>:128' */
      i = event_Moving_OK;
      chartstep_c2_PNC_StateFlow_2019(&i);
    }

    if (rtb_inputevents[12U] == 1)
    {
      /* Event: '<S1>:135' */
      i = event_Estop;
      chartstep_c2_PNC_StateFlow_2019(&i);
    }

    if (rtb_inputevents[13U] == 1)
    {
      /* Event: '<S1>:136' */
      i = event_Runing_Estop;
      chartstep_c2_PNC_StateFlow_2019(&i);
    }

    if (rtb_inputevents[14U] == 1)
    {
      /* Event: '<S1>:137' */
      i = event_Break;
      chartstep_c2_PNC_StateFlow_2019(&i);
    }

    if (rtb_inputevents[15U] == 1)
    {
      /* Event: '<S1>:138' */
      i = event_Runing_Break;
      chartstep_c2_PNC_StateFlow_2019(&i);
    }

    if (rtb_inputevents[16U] == 1)
    {
      /* Event: '<S1>:139' */
      i = event_Reset;
      chartstep_c2_PNC_StateFlow_2019(&i);
    }

    if (rtb_inputevents[17U] == 1)
    {
      /* Event: '<S1>:140' */
      i = event_Bypass;
      chartstep_c2_PNC_StateFlow_2019(&i);
    }

    if (rtb_inputevents[18U] == 1)
    {
      /* Event: '<S1>:141' */
      i = event_Stop_ok;
      chartstep_c2_PNC_StateFlow_2019(&i);
    }
  }
}

/* Model initialize function */
void PNC_StateFlow_20190412_2_initialize(void)
{
  {
    int32_T i;
    for (i = 0; i < 19; i++)
    {
      rtPrevZCX.PNC_Trig_ZCE[i] = UNINITIALIZED_ZCSIG;
    }

    /* SystemInitialize for Chart: '<Root>/PNC' */
    /* Entry: PNC */
    /* Entry Internal: PNC */
    /* Transition: '<S1>:12' */
    rtDW.is_c2_PNC_StateFlow_20190412_2 = IN_Standby;

    /* SystemInitialize for Outport: '<Root>/PNC_SF_Status' incorporates:
     *  SystemInitialize for Chart: '<Root>/PNC'
     */
    /* Entry 'Standby': '<S1>:42' */
    rtY.PNC_SF_Status = 1;
  }
}

/* Model terminate function */
void PNC_StateFlow_20190412_2_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */