#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="pidmotor.h" />
///
/// <summary>
///     This module contains the library functions for the PID motor
///     subsystem.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _PIDMOTOR_H
#define _PIDMOTOR_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                          MOD_PIDMOTOR

//
// Constants.
//
#define PIDMOTORF_PIDMODE_ON            0x0001
#define PIDMOTORF_HOLD_TARGET           0x0002
#define PIDMOTORF_USE_2PIDCTRL          0x0004
#define PIDMOTORF_STALLED               0x0008
#define PIDMOTORF_ON_TARGET             0x0010
#define PIDMOTORF_CALIBRATING           0x8000

#define PIDMOTORO_MOTOR1_REVERSED       0x0001
#define PIDMOTORO_MOTOR2_REVERSED       0x0002

//
// Type definitions.
//
typedef struct
{
    tMotor          motor1;
    tMotor          motor2;
    PIDCTRL        *pidCtrl1;
    PIDCTRL        *pidCtrl2;
    int             options;
    TOUCH          *lowerTouch;
    TOUCH          *upperTouch;
    SM             *sm;
    int             evtType;
    int             flags;
    int             motorPower;
    unsigned long   expiredTime;
    int             prevEncoder;
    unsigned long   prevTime;
    int             calPower;
    int             stallMinPower;
    unsigned long   stallTimeout;
} PIDMOTOR;

/**
 *  This macro checks if the PID motor is on target.
 *
 *  @param p Points to the PIDMOTOR structure to be reset.
 *
 *  @return Returns true if the set target has been reached, false otherwise.
 */
#define PIDMotorOnTarget(p)             ((p).flags & PIDMOTORF_ON_TARGET)

/**
 *  This function resets the PID motor system.
 *
 *  @param pidMotor Points to the PIDMOTOR structure to be reset.
 */
void
PIDMotorReset(
    PIDMOTOR &pidMotor
    )
{
    TFuncName("PIDMotorReset");
    TLevel(API);
    TEnter();

    motor[pidMotor.motor1] = 0;
    if (pidMotor.motor2 != (tMotor)-1)
    {
        motor[pidMotor.motor2] = 0;
    }

    if (pidMotor.pidCtrl1 != NULL)
    {
        PIDCtrlReset(*pidMotor.pidCtrl1);
    }

    if (pidMotor.pidCtrl2 != NULL)
    {
        PIDCtrlReset(*pidMotor.pidCtrl2);
    }

    pidMotor.sm = NULL;
    pidMotor.evtType = 0;
    pidMotor.motorPower = 0;
    pidMotor.expiredTime = 0;
    pidMotor.prevEncoder = 0;
    pidMotor.prevTime = 0;
    pidMotor.flags = 0;

    TExit();
    return;
}   //PIDMotorReset

/**
 *  This function initializes the PID motor system.
 *
 *  @param pidMotor Points to the PIDMOTOR structure.
 *  @param motorID Specifies the motor ID.
 *  @param pidCtrl1 Specifies the PID controller 1.
 *  @param options Specifies the PID motor options.
 *         PIDMOTORO_MOTOR1_REVERSED Specifies motor 1 is reversed.
 *         PIDMOTORO_MOTOR2_REVERSED Specifies motor 2 is reversed.
 *  @param lowerTouch Optionally specifies the lower touch switch.
 *  @param upperTouch Optionally specifies the upper touch switch.
 */
void
PIDMotorInit(
    PIDMOTOR &pidMotor,
    tMotor motorID,
    PIDCTRL &pidCtrl1,
    int options = 0,
    TOUCH *lowerTouch = NULL,
    TOUCH *upperTouch = NULL
    )
{
    TFuncName("PIDMotorInit");
    TLevel(INIT);
    TEnter();

    pidMotor.motor1 = motorID;
    pidMotor.motor2 = (tMotor)-1;
    pidMotor.pidCtrl1 = &pidCtrl1;
    pidMotor.pidCtrl2 = NULL;
    pidMotor.options = options;
    pidMotor.lowerTouch = lowerTouch;
    pidMotor.upperTouch = upperTouch;
    pidMotor.sm = NULL;
    pidMotor.evtType = 0;
    pidMotor.motorPower = 0;
    pidMotor.expiredTime = 0;
    pidMotor.prevEncoder = 0;
    pidMotor.prevTime = 0;
    pidMotor.flags = 0;
    pidMotor.calPower = 0;
    pidMotor.stallMinPower = 0;
    pidMotor.stallTimeout = 0;
    nMotorEncoder[motorID] = 0;

    TExit();
    return;
}   //PIDMotorInit

/**
 *  This function initializes the PID motor system.
 *
 *  @param pidMotor Points to the PIDMOTOR structure.
 *  @param motor1 Specifies the ID of motor 1.
 *  @param motor2 Specifies the ID of motor 2.
 *  @param pidCtrl1 Specifies the PID controller 1.
 *  @param options Specifies the PID motor options.
 *         PIDMOTORO_MOTOR1_REVERSED Specifies motor 1 is reversed.
 *         PIDMOTORO_MOTOR2_REVERSED Specifies motor 2 is reversed.
 *  @param lowerTouch Optionally specifies the lower touch switch.
 *  @param upperTouch Optionally specifies the upper touch switch.
 */
void
PIDMotorInit(
    PIDMOTOR &pidMotor,
    tMotor motor1,
    tMotor motor2,
    PIDCTRL &pidCtrl1,
    int options = 0,
    TOUCH *lowerTouch = NULL,
    TOUCH *upperTouch = NULL
    )
{
    TFuncName("PIDMotorInit");
    TLevel(INIT);
    TEnter();

    PIDMotorInit(pidMotor, motor1, pidCtrl1, options, lowerTouch, upperTouch);
    pidMotor.motor2 = motor2;

    TExit();
    return;
}   //PIDMotorInit

/**
 *  This function initializes the PID motor system.
 *
 *  @param pidMotor Points to the PIDMOTOR structure.
 *  @param motorID Specifies the motor ID.
 *  @param pidCtrl1 Specifies the PID controller 1.
 *  @param pidCtrl2 Specifies the PID controller 2.
 *  @param options Specifies the PID motor options.
 *         PIDMOTORO_MOTOR1_REVERSED Specifies motor 1 is reversed.
 *         PIDMOTORO_MOTOR2_REVERSED Specifies motor 2 is reversed.
 *  @param lowerTouch Optionally specifies the lower touch switch.
 *  @param upperTouch Optionally specifies the upper touch switch.
 */
void
PIDMotorInit(
    PIDMOTOR &pidMotor,
    tMotor motorID,
    PIDCTRL &pidCtrl1,
    PIDCTRL &pidCtrl2,
    int options = 0,
    TOUCH *lowerTouch = NULL,
    TOUCH *upperTouch = NULL
    )
{
    TFuncName("PIDMotorInit");
    TLevel(INIT);
    TEnter();

    PIDMotorInit(pidMotor, motorID, pidCtrl1, options, lowerTouch, upperTouch);
    pidMotor.pidCtrl2 = &pidCtrl2;

    TExit();
    return;
}   //PIDMotorInit

/**
 *  This function initializes the PID motor system.
 *
 *  @param pidMotor Points to the PIDMOTOR structure.
 *  @param motor1 Specifies the ID of motor 1.
 *  @param motor2 Specifies the ID of motor 2.
 *  @param pidCtrl1 Specifies the PID controller 1.
 *  @param pidCtrl2 Specifies the PID controller 2.
 *  @param options Specifies the PID motor options.
 *         PIDMOTORO_MOTOR1_REVERSED Specifies motor 1 is reversed.
 *         PIDMOTORO_MOTOR2_REVERSED Specifies motor 2 is reversed.
 *  @param lowerTouch Optionally specifies the lower touch switch.
 *  @param upperTouch Optionally specifies the upper touch switch.
 */
void
PIDMotorInit(
    PIDMOTOR &pidMotor,
    tMotor motor1,
    tMotor motor2,
    PIDCTRL &pidCtrl1,
    PIDCTRL &pidCtrl2,
    int options = 0,
    TOUCH *lowerTouch = NULL,
    TOUCH *upperTouch = NULL
    )
{
    TFuncName("PIDMotorInit");
    TLevel(INIT);
    TEnter();

    PIDMotorInit(pidMotor, motor1, motor2, pidCtrl1, options,
                 lowerTouch, upperTouch);
    pidMotor.pidCtrl2 = &pidCtrl2;

    TExit();
    return;
}   //PIDMotorInit

/**
 *  This function sets PID motor power.
 *
 *  @param pidMotor Points to the PIDMOTOR structure.
 *  @param power Specifies the power level to set the motor.
 *  @param stallMinPower Optionally specifies stall detection minimum power.
 *  @param stallTimeout Optionally specifies stall detection timeout.
 *  @param resetTimeout Optionally specifies stall reset timeout.
 *  @param fTask Optionally specifies true if the caller is the PIDMotor task.
 */
void
PIDMotorSetPower(
    PIDMOTOR &pidMotor,
    int power,
    int stallMinPower = 0,
    unsigned long stallTimeout = 0,
    unsigned long resetTimeout = 0,
    bool fTask = false
    )
{
    TFuncName("PIDMotorSetPower");
    TLevel(API);
    TEnterMsg(("power=%d", power));

    if (!fTask && (pidMotor.flags & PIDMOTORF_PIDMODE_ON))
    {
        //
        // There was a previous unfinished PID operation, cancel it.
        //
        PIDMotorReset(pidMotor);
    }

    power = BOUND(power, MOTOR_MIN_VALUE, MOTOR_MAX_VALUE);
    if (pidMotor.flags & PIDMOTORF_STALLED)
    {
        if (power == 0)
        {
            //
            // Clear the stall condition if power is zero.
            //
            if ((resetTimeout == 0) ||
                (GetMsecTime() - pidMotor.prevTime > resetTimeout))
            {
                pidMotor.prevEncoder = nMotorEncoder[pidMotor.motor1];
                pidMotor.prevTime = GetMsecTime();
                pidMotor.flags &= ~PIDMOTORF_STALLED;
            }
        }
        else
        {
            //
            // Beep as long as power is still applied while stalled.
            //
            pidMotor.prevTime = GetMsecTime();
            PlayImmediateTone(1000, 10);
            if (pidMotor.flags & PIDMOTORF_CALIBRATING)
            {
                //
                // We stalled during calibration, so exit calibration and
                // stop the beeping.
                //
                pidMotor.flags &= ~PIDMOTORF_CALIBRATING;
            }
        }
    }
    else
    {
        bool reachLowerLimit = (pidMotor.lowerTouch != NULL) &&
                               (power < 0) &&
                               TouchGetState(*pidMotor.lowerTouch);
        bool reachUpperLimit = (pidMotor.upperTouch != NULL) &&
                               (power > 0) &&
                               TouchGetState(*pidMotor.upperTouch);

        if (reachLowerLimit || reachUpperLimit)
        {
            //
            // We are hitting the limit switch, do not move further.
            //
            power = 0;
            if ((pidMotor.flags & PIDMOTORF_CALIBRATING) && reachLowerLimit)
            {
                //
                // Done calibrating.
                //
                pidMotor.flags &= ~PIDMOTORF_CALIBRATING;
                nMotorEncoder[pidMotor.motor1] = 0;
            }
        }

        pidMotor.motorPower = power;
        motor[pidMotor.motor1] =
            (pidMotor.options & PIDMOTORO_MOTOR1_REVERSED)? -power: power;
        if (pidMotor.motor2 != (tMotor)-1)
        {
            motor[pidMotor.motor2] =
               (pidMotor.options & PIDMOTORO_MOTOR2_REVERSED)?
                    -power: power;
        }

        if ((stallMinPower > 0) && (stallTimeout > 0))
        {
            //
            // Stall protection is ON, check for stall condition.
            // - power is above stallMinPower
            // - motor has not moved for at least stallTimeout
            //
            if ((abs(power) < abs(stallMinPower)) ||
                (nMotorEncoder[pidMotor.motor1] != pidMotor.prevEncoder))
            {
                pidMotor.prevEncoder = nMotorEncoder[pidMotor.motor1];
                pidMotor.prevTime = GetMsecTime();
            }

            if ((pidMotor.prevTime > 0) &&
                (GetMsecTime() - pidMotor.prevTime > stallTimeout))
            {
                //
                // We have detected a stalled condition for at least
                // stallMinTime. Kill the power to protect the motor.
                //
                motor[pidMotor.motor1] = 0;
                if (pidMotor.motor2 != (tMotor)-1)
                {
                    motor[pidMotor.motor2] = 0;
                }
                pidMotor.flags |= PIDMOTORF_STALLED;
            }
        }
    }

    TExit()
    return;
}   //PIDMotorSetPower

/**
 *  This function performs zero point calibration.
 *
 *  @param pidMotor Points to the PIDMOTOR structure.
 *  @param calPower Specifies the motor power for the calibration.
 *  @param stallMinPower Specifies the minimum power to detect stall condition.
 *  @param stallTimeout Specifies the minimum timeout for stall condition.
 *  @param fWait Optionally specifies if the function will wait until
 *         calibration is done.
 */
void
PIDMotorZeroCalibrate(
    PIDMOTOR &pidMotor,
    int calPower,
    int stallMinPower,
    unsigned long stallTimeout,
    bool fWait = true
    )
{
    TFuncName("PIDMotorZeroCalibrate");
    TLevel(API);
    TEnterMsg(("power=%d,timeout=%d", calPower, stallTimeout));

    pidMotor.flags |= PIDMOTORF_CALIBRATING;
    pidMotor.calPower = calPower;
    pidMotor.stallMinPower = stallMinPower;
    pidMotor.stallTimeout = stallTimeout;
    do
    {
        PIDMotorSetPower(pidMotor,
                         calPower,
                         stallMinPower,
                         stallTimeout);
        wait1Msec(20);
    } while (fWait && (pidMotor.flags & PIDMOTORF_CALIBRATING));

    TExit();
    return;
}   //PIDMotorZeroCalibrate

/**
 *  This function sets PID motor target.
 *
 *  @param pidMotor Points to the PIDMOTOR structure.
 *  @param setPoint Specifies the set motor target.
 *  @param fHoldTarget Optionally specifies if PIDMotor will maintain target
 *         or will stop when target is reached.
 *  @param sm Optionally points to the state machine that needs completion
 *         notification.
 *  @param evtType Optionally specifies the event type to be signed when
 *         PID operation is completed. Required only if sm is not NULL.
 *  @param timeout Optionally specifies the timeout period.
 */
void
PIDMotorSetTarget(
    PIDMOTOR &pidMotor,
    float setPoint,
    bool fHoldTarget = false,
    SM *sm = NULL,
    int evtType = 0,
    unsigned long timeout = 0
    )
{
    TFuncName("PIDMotorSetTarget");
    TLevel(API);
    TEnterMsg(("setPt=%5.1f", setPoint));

    pidMotor.sm = sm;
    pidMotor.evtType = evtType;
    if (pidMotor.flags & PIDMOTORF_PIDMODE_ON)
    {
        //
        // Previous SetTarget has not been completed, cancel it.
        //
        PIDMotorReset(pidMotor);
    }

    PIDCtrlSetTarget(*pidMotor.pidCtrl1,
                     setPoint,
                     PIDCtrlGetInput(*pidMotor.pidCtrl1));
    pidMotor.expiredTime = (timeout != 0)? GetMsecTime() + timeout: 0;

    if (fHoldTarget)
    {
        pidMotor.flags |= PIDMOTORF_HOLD_TARGET;
    }
    else
    {
        pidMotor.flags &= ~PIDMOTORF_HOLD_TARGET;
    }

    pidMotor.flags &= ~PIDMOTORF_USE_2PIDCTRL;
    pidMotor.flags |= PIDMOTORF_PIDMODE_ON;
    pidMotor.flags &= ~PIDMOTORF_ON_TARGET;

    TExit();
    return;
}   //PIDMotorSetTarget

/**
 *  This function sets PID motor target.
 *
 *  @param pidMotor Points to the PIDMOTOR structure.
 *  @param setPoint1 Specifies the set motor target 1.
 *  @param setPoint2 Specifies the set motor target 2.
 *  @param fHoldTarget Optionally specifies if PIDMotor will maintain target
 *         or will stop when target is reached.
 *  @param sm Optionally points to the state machine that needs completion
 *         notification.
 *  @param evtType Optionally specifies the event type to be signed when
 *         PID operation is completed. Required only if sm is not NULL.
 *  @param timeout Optionally specifies the timeout period.
 */
void
PIDMotorSetTarget(
    PIDMOTOR &pidMotor,
    float setPoint1,
    float setPoint2,
    bool fHoldTarget = false,
    SM *sm = NULL,
    int evtType = 0,
    unsigned long timeout = 0
    )
{
    TFuncName("PIDMotorSetTarget");
    TLevel(API);
    TEnterMsg(("SP1=%5.1f,SP2=%5.1f", setPoint1, setPoint2));

    pidMotor.sm = sm;
    pidMotor.evtType = evtType;
    if (pidMotor.flags & PIDMOTORF_PIDMODE_ON)
    {
        //
        // Previous SetTarget has not been completed, cancel it.
        //
        PIDMotorReset(pidMotor);
    }

    PIDCtrlSetTarget(*pidMotor.pidCtrl1,
                     setPoint1,
                     PIDCtrlGetInput(*pidMotor.pidCtrl1));
    if (setPoint2 != 0.0)
    {
        PIDCtrlSetTarget(*pidMotor.pidCtrl2,
                         setPoint2,
                         PIDCtrlGetInput(*pidMotor.pidCtrl2));
    }

    pidMotor.expiredTime = (timeout != 0)? GetMsecTime() + timeout: 0;

    if (fHoldTarget)
    {
        pidMotor.flags |= PIDMOTORF_HOLD_TARGET;
    }
    else
    {
        pidMotor.flags &= ~PIDMOTORF_HOLD_TARGET;
    }

    if (setPoint2 != 0.0)
    {
        pidMotor.flags |= PIDMOTORF_USE_2PIDCTRL;
    }
    else
    {
        pidMotor.flags &= ~PIDMOTORF_USE_2PIDCTRL;
    }
    pidMotor.flags |= PIDMOTORF_PIDMODE_ON;
    pidMotor.flags &= ~PIDMOTORF_ON_TARGET;

    TExit();
    return;
}   //PIDMotorSetTarget

/**
 *  This function performs the PID motor task.
 *
 *  @param pidMotor Points to the PIDMOTOR structure.
 */
void
PIDMotorTask(
    PIDMOTOR &pidMotor
    )
{
    TFuncName("PIDMotorTask");
    TLevel(TASK);
    TEnter();

    if (pidMotor.flags & PIDMOTORF_CALIBRATING)
    {
        PIDMotorSetPower(pidMotor,
                         pidMotor.calPower,
                         pidMotor.stallMinPower,
                         pidMotor.stallTimeout);
    }
    else if (pidMotor.flags & PIDMOTORF_PIDMODE_ON)
    {
        int power1 = (int)PIDCtrlOutput(*pidMotor.pidCtrl1,
                                        PIDCtrlGetInput(*pidMotor.pidCtrl1));
        int power2 = 0;

        if (pidMotor.flags & PIDMOTORF_USE_2PIDCTRL)
        {
            power2 = (int)PIDCtrlOutput(*pidMotor.pidCtrl2,
                                        PIDCtrlGetInput(*pidMotor.pidCtrl2));
            pidMotor.motorPower = min(power1, power2);
        }
        else
        {
            pidMotor.motorPower = power1;
        }

        if ((pidMotor.expiredTime != 0) &&
            (GetMsecTime() >= pidMotor.expiredTime) ||
            PIDCtrlIsOnTarget(*pidMotor.pidCtrl1))
        {
            if (!(pidMotor.flags & PIDMOTORF_HOLD_TARGET))
            {
                if (pidMotor.sm != NULL)
                {
                    SMSetEvent(*pidMotor.sm, pidMotor.evtType);
                }
                PIDMotorReset(pidMotor);
            }
            pidMotor.flags |= PIDMOTORF_ON_TARGET;
        }

        if (pidMotor.flags & PIDMOTORF_PIDMODE_ON)
        {
            //
            // PID mode is still ON. Either we haven't rearched target yet
            // or we are holding target.
            //
            PIDMotorSetPower(pidMotor, pidMotor.motorPower, 0, 0, 0, true);
        }
    }

    TExit();
    return;
}   //PIDMotorTask

#endif  //ifndef _PIDMOTOR_H
