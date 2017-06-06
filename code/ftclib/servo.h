#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="servo.h" />
///
/// <summary>
///     This module contains the library functions for the servo subsystem.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _SERVO_H
#define _SERVO_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_SERVO

//
// Constants.
//
#define SERVOO_SERVO1_REVERSED          0x0001
#define SERVOO_SERVO2_REVERSED          0x0002

#define SERVOF_CONTINUOUS_ENABLED       0x0001
#define SERVOF_CONTINUOUS               0x0002
#define SERVOF_STEPPING                 0x0004

#define DEF_MIN_VALUE                   0
#define DEF_MAX_VALUE                   255
#define DEF_MAX_ANGLE                   180.0   //degrees
#define DEF_ZERO_ANGLE                  0.0     //degrees
#define DEF_MAX_STEP_RATE               90.0    //degrees/sec

#define SERVO_CONTINUOUS_STOP           127
#define SERVO_CONTINUOUS_FWD_MAX        255
#define SERVO_CONTINUOUS_REV_MAX        0
#define SERVO_CONTINUOUS_LOW            -127
#define SERVO_CONTINUOUS_HIGH           128

//
// Macros.
//

/**
 *  This macro returns the current servo angle if it is in stepping mode.
 *  Otherwise, it returns zero.
 *
 *  @param s Points to the SERVO structure.
 *
 *  @return Returns the current servo angle if in stepping mode.
 */
#define ServoGetAngle(s)        (((s).flags & SERVOF_CONTINUOUS)?       \
                                 0.0: (s).currAngle)

/**
 *  This macro returns the current power if it is a continuous servo.
 *
 *  @param s Points to the SERVO structure.
 *
 *  @return Returns the current power if it is a continuous servo.
 */
#define ServoGetPower(s)        (((s).flags & SERVOF_CONTINUOUS)?       \
                                 (s).power: 0)

//
// Type definitions.
//
typedef struct
{
    TServoIndex     servo1;
    TServoIndex     servo2;
    float           minAngle;
    float           maxAngle;
    float           zeroAngle;
    float           maxStepRate;
    int             minValue;
    int             maxValue;
    int             options;
    int             flags;
    float           currAngle;
    float           targetAngle;
    float           stepRate;
    unsigned long   prevTime;
    //
    // The following is for continuous servo.
    //
    tSensors        lowerTouchID;
    tSensors        upperTouchID;
    int             power;
} SERVO;

/**
 *  This function sets servo position in degrees.
 *
 *  @param serv Points to the SERVO structure.
 *  @param angle Specifies the servo position in degrees.
 *  @param stepRate Optionally specifies the step rate in degrees/sec.
 */
void
ServoSetAngle(
    SERVO &serv,
    float angle,
    float stepRate = 0.0
    )
{
    TFuncName("ServoSetAngle");
    TLevel(API);
    TEnterMsg(("A=%5.1f,R=%5.1f", angle, stepRate));

    //
    // This API is not applicable for continuous servo.
    //
    if (!(serv.flags & SERVOF_CONTINUOUS))
    {
        angle = BOUND(angle, serv.minAngle, serv.maxAngle);
        serv.stepRate = abs(stepRate);
        if (stepRate == 0.0)
        {
            serv.currAngle = angle;
            serv.flags &= ~SERVOF_STEPPING;
        }
        else
        {
            serv.prevTime = nPgmTime;
            serv.targetAngle = angle;
            serv.flags |= SERVOF_STEPPING;
        }
    }

    TExit();
    return;
}   //ServoSetAngle

/**
 *  This function initializes a standard servo motor.
 *
 *  @param serv Points to the SERVO structure.
 *  @param servoID Specifies the servo motor ID.
 *  @param options Optionally specifies the servo options:
 *         SERVOO_SERVO1_REVERSED - Servo 1 is reversed.
 *  @param maxAngle Optionally specifies the maximum servo angle.
 *  @param zeroAngle Optionally specifies the zero angle.
 *  @param maxStepRate Optionally specifies the maximum step rate for stepping.
 *  @param minValue Optionally specifies the minimum servo value that is
 *         outside of the left deadband.
 *  @param maxValue Optionally specifies the maximum servo value that is
 *         outside of the right deadband.
 */
void
ServoInit(
    SERVO &serv,
    TServoIndex servoID,
    int options = 0,
    float maxAngle = DEF_MAX_ANGLE,
    float zeroAngle = DEF_ZERO_ANGLE,
    float maxStepRate = DEF_MAX_STEP_RATE,
    int minValue = DEF_MIN_VALUE,
    int maxValue = DEF_MAX_VALUE
    )
{
    TFuncName("ServoInit");
    TLevel(INIT);
    TEnter();

    serv.servo1 = servoID;
    serv.servo2 = (TServoIndex)-1;
    serv.options = options;
    serv.minAngle = -zeroAngle;
    serv.maxAngle = maxAngle - zeroAngle;
    serv.maxStepRate = maxStepRate;
    serv.minValue = minValue;
    serv.maxValue = maxValue;
    serv.flags = 0;
    serv.currAngle = 0.0;
    serv.targetAngle = 0.0;
    serv.stepRate = 0.0;
    serv.prevTime = 0;
    //
    // The following is for continuous servo.
    //
    serv.lowerTouchID = (tSensors)-1;
    serv.upperTouchID = (tSensors)-1;
    serv.power = 0;

    TExit();
    return;
}   //ServoInit

/**
 *  This function initializes a standard servo motor.
 *
 *  @param serv Points to the SERVO structure.
 *  @param servo1ID Specifies the servo motor 1 ID.
 *  @param servo2ID Specifies the servo motor 2 ID.
 *  @param options Optionally specifies the servo options:
 *         SERVOO_SERVO1_REVERSED - Servo 1 is reversed.
 *         SERVOO_SERVO2_REVERSED - Servo 2 is reversed.
 *  @param maxAngle Optionally specifies the maximum servo angle.
 *  @param zeroAngle Optionally specifies the zero angle.
 *  @param maxStepRate Optionally specifies the maximum step rate for SetPower.
 *  @param minValue Optionally specifies the minimum servo value that is
 *         outside of the left deadband.
 *  @param maxValue Optionally specifies the maximum servo value that is
 *         outside of the right deadband.
 */
void
ServoInit(
    SERVO &serv,
    TServoIndex servo1ID,
    TServoIndex servo2ID,
    int options = 0,
    float maxAngle = DEF_MAX_ANGLE,
    float zeroAngle = DEF_ZERO_ANGLE,
    float maxStepRate = DEF_MAX_STEP_RATE,
    int minValue = DEF_MIN_VALUE,
    int maxValue = DEF_MAX_VALUE
    )
{
    TFuncName("ServoInit");
    TLevel(INIT);
    TEnter();

    ServoInit(serv,
              servo1ID,
              options,
              maxAngle,
              zeroAngle,
              maxStepRate,
              minValue,
              maxValue);
    serv.servo2 = servo2ID;

    TExit();
    return;
}   //ServoInit

/**
 *  This function initializes a continuous servo motor.
 *
 *  @param serv Points to the SERVO structure.
 *  @param servoID Specifies the servo motor ID.
 *  @param lowerTouchID Optionally specifies the sensor ID for the lower
 *         limit switch.
 *  @param upperTouchID Optionally specifies the sensor ID for the upper
 *         limit switch.
 */
void
ServoInit(
    SERVO &serv,
    TServoIndex servoID,
    tSensors lowerTouchID = -1,
    tSensors upperTouchID = -1
    )
{
    TFuncName("ServoInit");
    TLevel(INIT);
    TEnter();

    ServoInit(serv,
              servoID,
              -1.0);
    serv.flags = SERVOF_CONTINUOUS;
    serv.lowerTouchID = lowerTouchID;
    serv.upperTouchID = upperTouchID;
    serv.power = SERVO_CONTINUOUS_STOP;

    TExit();
    return;
}   //ServoInit

/**
 *  This function sets servo position in degrees.
 *
 *  @param serv Points to the SERVO structure.
 *  @param power Specifies the continuous servo power.
 */
void
ServoContinuousSetPower(
    SERVO &serv,
    int power
    )
{
    TFuncName("ServoContSetPower");
    TLevel(API);
    TEnterMsg(("power=%5.1f", power));

    if (serv.flags & SERVOF_CONTINUOUS)
    {
        serv.power = NORMALIZE(BOUND(power, -100, 100),
                               -100, 100,
                               DEF_MIN_VALUE, DEF_MAX_VALUE);

        servo[serv.servo1] = serv.power;
        serv.flags |= SERVOF_CONTINUOUS_ENABLED;
    }

    TExit();
    return;
}   //ServoContinuousSetPower

/**
 *  This function stops the servo and hold its position.
 *
 *  @param serv Points to the SERVO structure.
 */
void
ServoStop(
    SERVO &serv
    )
{
    TFuncName("ServoStop");
    TLevel(API);
    TEnter();

    if (serv.flags & SERVOF_CONTINUOUS)
    {
        serv.power = SERVO_CONTINUOUS_STOP;
        servo[serv.servo1] = serv.power;
        serv.flags &= ~SERVOF_CONTINUOUS_ENABLED;
    }
    else if (serv.flags & SERVOF_STEPPING)
    {
        serv.flags &= ~SERVOF_STEPPING;
    }

    TExit();
    return;
}   //ServoStop

/**
 *  This function performs the servo task.
 *
 *  @param serv Points to the SERVO structure.
 *  @param lineNum Optionally specifies the debugging LCD line number.
 */
void
ServoTask(
    SERVO &serv,
    int lineNum = -1
    )
{
    TFuncName("ServoTask");
    TLevel(TASK);
    TEnter();

    if (serv.flags & SERVOF_CONTINUOUS_ENABLED)
    {
        if ((serv.lowerTouchID != -1) &&
            (SensorValue[serv.lowerTouchID] == 1) ||
            (serv.upperTouchID != -1) &&
            (SensorValue[serv.upperTouchID] == 1))
        {
            //
            // We hit one of the limit switches, so stop!
            //
            servo[serv.servo1] = SERVO_CONTINUOUS_STOP;
            serv.flags &= ~SERVOF_CONTINUOUS_ENABLED;
        }
    }
    else
    {
        if (serv.flags & SERVOF_STEPPING)
        {
            unsigned long currTime = nPgmTime;
            float stepAngle = serv.stepRate*(currTime - serv.prevTime)/1000.0;

            if (serv.currAngle < serv.targetAngle)
            {
                serv.currAngle += stepAngle;
                if (serv.currAngle > serv.targetAngle)
                {
                    serv.currAngle = serv.targetAngle;
                }
            }
            else if (serv.currAngle > serv.targetAngle)
            {
                serv.currAngle -= stepAngle;
                if (serv.currAngle < serv.targetAngle)
                {
                    serv.currAngle = serv.targetAngle;
                }
            }
            else
            {
                //
                // We have reached the target angle, stop stepping.
                //
                serv.flags &= ~SERVOF_STEPPING;
            }

            serv.prevTime = currTime;
        }

        int servoPos = round((serv.currAngle - serv.minAngle)*
                             (serv.maxValue - serv.minValue)/
                             (serv.maxAngle - serv.minAngle) +
                             serv.minValue);
        if (lineNum != -1)
        {
            nxtDisplayTextLine(lineNum, "A=%5.1f,P=%d",
                               serv.currAngle, servoPos);
        }

        if (serv.options & SERVOO_SERVO1_REVERSED)
        {
            servo[serv.servo1] = serv.maxValue - servoPos + serv.minValue;
        }
        else
        {
            servo[serv.servo1] = servoPos;
        }

        if (serv.servo2 != (TServoIndex)(-1))
        {
            if (serv.options & SERVOO_SERVO2_REVERSED)
            {
                servo[serv.servo2] = serv.maxValue - servoPos + serv.minValue;
            }
            else
            {
                servo[serv.servo2] = servoPos;
            }
        }
    }

    TExit();
    return;
}   //ServoTask

#endif  //ifndef _SERVO_H
