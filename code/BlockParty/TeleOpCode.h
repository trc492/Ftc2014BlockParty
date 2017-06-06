#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="TeleOpCode.h" />
///
/// <summary>
///     This module contains the TeleOp-only functions.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

//
// Include libraries.
//

// Include RobotC drivers.

// Include FTC library.
#include "..\ftclib\joystick.h"

//
// Global data.
//

// Input subsystem.
JOYSTICK    g_Joystick1;
JOYSTICK    g_Joystick2;

#ifdef _WILLIAM_ENDEFFECTOR
//William Subsystem
int         g_williamPos = WILLIAM_MID_POS;
#else
// Scoop subsystem.
bool        g_scoopUp = false;
bool        g_scoopDown = false;
float       g_scoopPos = SCOOP_UP_POS;
#endif

// Miscellaneous.
bool        g_slideOverride = false;
float       g_PrevSlideTarget = 0.0;
bool        g_noDrive = false;

bool        g_armOverride = false;
float       g_PrevArmTarget = 0.0;

//
// Callback functions.
//
float PIDCtrlModeGetInput(PIDCTRL &pidCtrl)
{
    float inputValue = 0.0;
    return inputValue;
}   //PIDCtrlModeGetInput

/**
 *  This function handles the joystick button notification events.
 *
 *  @param joystickID Specifies the joystick the event was from.
 *  @param eventType Specifies the event type.
 *  @param eventID Specifies the event ID.
 *  @param fPressed Specifies the event is a press or a release.
 */
void
JoyBtnEvent(
    int  joystickID,
    int  eventType,
    int  eventID,
    bool fPressed
    )
{
    if (joystickID == 1)
    {
        if (eventType == JOYEVENT_BUTTON)
        {
            switch (eventID)
            {
                case Logitech_Btn1:
                    break;

                case Logitech_LB7:
                    break;

                case Logitech_RB8:
                    g_noDrive = fPressed;
                    break;

                case Logitech_Btn9:
                    break;

                case Logitech_Btn10:
                    break;

                default:
                    break;
            }
        }
        else if (eventType == JOYEVENT_TOPHAT)
        {
            switch (eventID)
            {
                case TopHat_Up:
                    break;

                case TopHat_Down:
                    break;

                default:
                    break;
            }
        }
    }
    else if (joystickID == 2)
    {
        if (eventType == JOYEVENT_BUTTON)
        {
            switch (eventID)
            {
                case Logitech_Btn1:
                    break;

                case Logitech_Btn2:
                    if(fPressed)
                    {
                        motor[crankMotor] = -CRANK_POWER;
                    }
                    else
                    {
                        motor[crankMotor] = 0;
                    }
                    break;

                case Logitech_Btn3:
                    break;

                case Logitech_Btn4:
                    if(fPressed)
                    {
                        motor[crankMotor] = CRANK_POWER;
                    }
                    else
                    {
                        motor[crankMotor] = 0;
                    }
                    break;

                case Logitech_LB7:
                    g_slideOverride = fPressed;
                    break;

                case Logitech_RB8:
                    g_armOverride = fPressed;
                    break;

                case Logitech_Btn9:
#ifndef DISABLE_SLIDE_ZERO
                    PIDMotorZeroCalibrate(g_slideMotor,
                                          SLIDE_CAL_POWER,
                                          SLIDE_STALL_MINPOWER,
                                          SLIDE_STALL_TIMEOUT,
                                          false);
#else
                    //do nothing
#endif
                    break;

                case Logitech_Btn10:
                    PIDMotorZeroCalibrate(g_armMotor,
                                          ARM_CAL_POWER,
                                          ARM_STALL_MINPOWER,
                                          ARM_STALL_TIMEOUT,
                                          false);
                    break;

                default:
                    break;
            }
        }
        else if (eventType == JOYEVENT_TOPHAT)
        {
            switch (eventID)
            {
#ifdef _WILLIAM_ENDEFFECTOR
                case TopHat_Up:
                    g_williamPos = WILLIAM_UP_POS;
                    break;

                case TopHat_Down:
                    g_williamPos = WILLIAM_DOWN_POS;
                    break;

                case TopHat_Left:
                case TopHat_Right:
                    g_williamPos = WILLIAM_MID_POS;
                    break;

                default:
                    break;


#else
                case TopHat_Up:
                    g_scoopUp = true;
                    break;

                case TopHat_Down:
                    g_scoopDown = true;
                    break;

                case TopHat_Left:
                    break;

                case TopHat_Right:
                    break;

                default:
                    g_scoopUp=false;
                    g_scoopDown=false;
                    break;
#endif
            }
        }
    }
}   //JoyBtnEvent

//
// Main functions.
//

/**
 *  This function is called once globally to do TeleOp specific
 *  initializations.
 */
void TeleOpInit()
{
    //
    // Initialize Input subsystem.
    //
    JoystickInit(g_Joystick1, 1);
    JoystickInit(g_Joystick2, 2);
}   //TeleOpInit

/**
 *  This function is called before TeleOp mode starts.
 */
void
TeleOpStart()
{
}   //TeleOpStart

/**
 *  This function is called after TeleOp mode ends.
 */
void
TeleOpStop()
{
}   //TeleOpStop

/**
 *  This function is called periodically to perform the TeleOp tasks.
 *
 *  @param time Specifies the running time since TeleOp started.
 */
void
TeleOpTasks(
    float time
    )
{
    nxtDisplayTextLine(0, "TeleOp [%5.1f]", time);

    //
    // Process Drive subsystem.
    //
    int x, y, rot;
    rot = JoystickGetX2WithDeadband(g_Joystick1, true);
    x   = JoystickGetX1WithDeadband(g_Joystick1, true);
    y   = JoystickGetY1WithDeadband(g_Joystick1, true);
    DriveSwerve(g_drive, x, y, rot, false, false, g_noDrive);

    //
    // Process Slide subsystem.
    //
    int slidePower = JoystickGetY1WithDeadband(g_Joystick2, true);

    if (TouchGetState(g_slideTouch) && (slidePower < 0))
    {
        //
        // We are completely retracted, so don't retract anymore.
        //
        slidePower = 0;
    }

    if (g_slideOverride)
    {
        PIDMotorSetPower(g_slideMotor,
                         slidePower,
                         SLIDE_STALL_MINPOWER,
                         SLIDE_STALL_TIMEOUT);
#ifdef _DEBUG_SLIDE
        nxtDisplayTextLine(3, "OP=%4d", slidePower);
#endif
    }
    else
    {
        //
        // Increase deadband to make sure the elevator motor doesn't get
        // stalled when joystick is not returned to zero.
        //
        float currTarget = (slidePower > 0)? SLIDE_UPPER_LIMIT:
                           (slidePower < 0)? SLIDE_LOWER_LIMIT: -1.0;
#ifdef _DEBUG_SLIDE
        nxtDisplayTextLine(3, "pT=%5.1f,cT=%5.1f", g_PrevSlideTarget, currTarget);
#endif
        if (currTarget != g_PrevSlideTarget)
        {
            if (slidePower == 0)
            {
                PIDMotorReset(g_slideMotor);
            }
            else
            {
                PIDMotorSetTarget(g_slideMotor, currTarget, false);
            }
            //nxtDisplayTextLine(4, "cT=%d, p=%d", currTarget, slidePower);
            g_PrevSlideTarget = currTarget;
        }
    }
#ifdef _DEBUG_SLIDE
    nxtDisplayTextLine(1, "L=%5.1f,E=%d",
                       PIDCtrlGetInput(g_slidePidCtrl),
                       nMotorEncoder[slideMotor1]);
    nxtDisplayTextLine(2, "JP=%4d", slidePower);
#endif

    //
    // Process Arm subsystem.
    //
    int armPower = JoystickGetY2WithDeadband(g_Joystick2, true);

    if (TouchGetState(g_armTouch) && (armPower < 0))
    {
        //
        // The arm is completely down, so don't move down anymore.
        //
        armPower = 0;
    }

    if (g_armOverride)
    {
        PIDMotorSetPower(g_armMotor,
                         armPower,
                         ARM_STALL_MINPOWER,
                         ARM_STALL_TIMEOUT);
#ifdef _DEBUG_ARM
        nxtDisplayTextLine(3, "OP=%4d", armPower);
#endif
    }
    else
    {
        //
        // Increase deadband to make sure the elevator motor doesn't get
        // stalled when joystick is not returned to zero.
        //
        float currTarget = (armPower > 0)? ARM_UPPER_LIMIT:
                           (armPower < 0)? ARM_LOWER_LIMIT: -1.0;
#ifdef _DEBUG_ARM
        nxtDisplayTextLine(3, "pT=%5.1f,cT=%5.1f", g_PrevArmTarget, currTarget);
#endif
        if (currTarget != g_PrevArmTarget)
        {
            if (armPower == 0)
            {
                PIDMotorReset(g_armMotor);
            }
            else
            {
                PIDMotorSetTarget(g_armMotor, currTarget, false);
            }
            g_PrevArmTarget = currTarget;
        }
    }
#ifdef _DEBUG_ARM
    nxtDisplayTextLine(1, "A=%5.1f,E=%d",
                       PIDCtrlGetInput(g_armPidCtrl),
                       nMotorEncoder[armMotor]);
    nxtDisplayTextLine(2, "JP=%4d", armPower);
#endif

#ifdef _WILLIAM_ENDEFFECTOR
    //
    //Process William Subsystem
    //
    ServoSetAngle(g_william, g_williamPos);
#else
    //
    // Process Scoop subsystem
    //
    if(g_scoopDown)
    {
        g_scoopPos-=SCOOP_INC;
        if(g_scoopPos<SCOOP_DOWN_POS)
        {
            g_scoopPos = SCOOP_DOWN_POS;
        }
    }
    else if(g_scoopUp)
    {
        g_scoopPos+=SCOOP_INC;
        if(g_scoopPos>SCOOP_UP_POS)
        {
            g_scoopPos = SCOOP_UP_POS;
        }
    }
    ServoSetAngle(g_scoop, g_scoopPos);
#endif

#ifdef _DEBUG_DRIVE
    nxtDisplayTextLine(1, "FL=%d,FR=%d",
                       nMotorEncoder[frontLeftMotor],
                       nMotorEncoder[frontRightMotor]);
    nxtDisplayTextLine(2, "RL=%d,RR=%d",
                       nMotorEncoder[rearLeftMotor],
                       nMotorEncoder[rearRightMotor]);
    nxtDisplayTextLine(3, "X=%6.1f,Y=%6.1f",
                       DriveGetXPos(g_drive), DriveGetYPos(g_drive));
    nxtDisplayTextLine(4, "Rot=%6.1f", DriveGetRotPos(g_drive));
#endif
//    nxtDisplayTextLine(4, "US=%d", USreadDist(sonarSensor));
}   //TeleOpTasks

//
// Other periodic tasks.
//

/**
 *  This function is called periodically to perform tasks specific to
 *  TeleOp mode that require high resolution (e.g. gyro integration).
 */
void TeleOpHiFreqTasks()
{
}   //TeleOpHiFreqTasks

/**
 *  This function is called periodically to perform tasks related to sensors
 *  and inputs that are specific to TeleOp mode.
 */
void TeleOpInputTasks()
{
    JoystickTask(g_Joystick1);
    JoystickTask(g_Joystick2);
}   //TeleOpInputTasks

/**
 *  This function is called periodically to perform tasks related to motors
 *  and other outputs that are specific to TeleOp mode.
 */
void TeleOpOutputTasks()
{
}   //TeleOpOutputTasks
