#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="CommonCode.h" />
///
/// <summary>
///     This module contains code common to both Autonomous and TeleOp modes.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

//
// Program switches.
//

// Common switches.
//#define _COMPETITION_ENABLED
//#define _WILLIAM_ENDEFFECTOR

// Autonomous switches.
#define _USE_RADAR

// TeleOp switches.

// Debug switches.
#ifndef _COMPETITION_ENABLED
#define _ENABLE_TESTMODES
//#define _DEBUG_SERVO
//#define _DEBUG_SLIDE
//#define _DEBUG_ARM
//#define _DEBUG_DRIVE
#endif

//
// Include libraries.
//

// Include RobotC drivers.
#include "..\RobotCDrivers\drivers\hitechnic-sensormux.h"
#include "..\RobotCDrivers\drivers\lego-ultrasound.h"
#include "..\RobotCDrivers\drivers\hitechnic-colour-v1.h"

// Include FTC library.
#include "..\ftclib\batt.h"
#include "..\ftclib\sm.h"
#include "..\ftclib\servo.h"
#include "..\ftclib\touch.h"
#include "..\ftclib\drive.h"
#include "..\ftclib\pidctrl.h"
#include "..\ftclib\pidmotor.h"

// Include local files.
#include "RobotInfo.h"

//
// Constants.
//

//
// Global data.
//

// Drive susbsystem.
SERVO           g_frontLeftServo;
SERVO           g_frontRightServo;
SERVO           g_rearLeftServo;
SERVO           g_rearRightServo;
DRIVE           g_drive;

// Slide subsystem
TOUCH           g_slideTouch;
PIDCTRL         g_slidePidCtrl;
PIDMOTOR        g_slideMotor;

// Arm subsystem.
TOUCH           g_armTouch;
PIDCTRL         g_armPidCtrl;
PIDMOTOR        g_armMotor;

#ifdef _WILLIAM_ENDEFFECTOR
//William subsystem
SERVO           g_william;
#else
//Scoop subsystem
SERVO           g_scoop;
#endif

// Miscellaneous.
BATT            g_batt;

//
// Callback functions.
//
float PIDCtrlModeGetInput(PIDCTRL &pidCtrl);
float PIDCtrlGetInput(PIDCTRL &pidCtrl)
{
    float inputValue = 0.0;

    if (PIDCtrlCheck(pidCtrl, g_slidePidCtrl))
    {
        inputValue = nMotorEncoder[slideMotor1]/SLIDE_CLICKS_PER_INCH;
    }
    else if (PIDCtrlCheck(pidCtrl, g_armPidCtrl))
    {
        inputValue = nMotorEncoder[armMotor]/ARM_CLICKS_PER_DEGREE;
    }
    else
    {
        inputValue = PIDCtrlModeGetInput(pidCtrl);
    }

    return inputValue;
}   //PIDCtrlGetInput

void TouchEvent(TOUCH &touch, bool fActive)
{
    if (&touch == &g_armTouch)
    {
	    if (fActive)
	    {
            //
            // We reached the bottom, stop the motor.
            //
            PIDMotorReset(g_armMotor);
            PlayImmediateTone(440, 15);
	    }
	    else
	    {
            PlayImmediateTone(880, 15);
	    }
	    //
	    // Reset the encoder when engaging or disengaging the touch sensor.
	    // This will compensate the small amount of free play on the chain
	    // where the motor is winding or unwinding the chain while the elevator
	    // is not yet moving.
	    //
	    nMotorEncoder[armMotor] = 0;
	}
	else if (&touch == &g_slideTouch)
    {
	    if (fActive)
	    {
            //
            // We reached the bottom, stop the motor.
            //
            PIDMotorReset(g_slideMotor);
            PlayImmediateTone(440, 15);
	    }
	    else
	    {
            PlayImmediateTone(880, 15);
	    }
	    //
	    // Reset the encoder when engaging or disengaging the touch sensor.
	    // This will compensate the small amount of free play on the chain
	    // where the motor is winding or unwinding the chain while the elevator
	    // is not yet moving.
	    //
	    nMotorEncoder[slideMotor1] = 0;
	}
}   //TouchEvent

//
// Main functions.
//

/**
 *  This function is called once globally to initialize the robot. Typically,
 *  this function initializes all the sensors and robot subsystems as well as
 *  global data that are common to both Autonomous and TeleOp modes.
 */
void
CommonInit()
{
    //
    // Initialize Drive subsystem.
    //
    ServoInit(g_frontLeftServo,
              frontLeftServo,
              0,
              SERVO_MAX_ANGLE,
              SERVO_FL_ZERO_ANGLE,
              SERVO_MAX_STEP_RATE,
              SERVO_MIN_VALUE, SERVO_MAX_ANGLE);
//              59, 194);
    ServoInit(g_frontRightServo,
              frontRightServo,
              0,
              SERVO_MAX_ANGLE,
              SERVO_FR_ZERO_ANGLE,
              SERVO_MAX_STEP_RATE,
              SERVO_MIN_VALUE, SERVO_MAX_ANGLE);
//              42, 185);
    ServoInit(g_rearLeftServo,
              rearLeftServo,
              0,
              SERVO_MAX_ANGLE,
              SERVO_RL_ZERO_ANGLE,
              SERVO_MAX_STEP_RATE,
              SERVO_MIN_VALUE, SERVO_MAX_ANGLE);
//              47, 190);
    ServoInit(g_rearRightServo,
              rearRightServo,
              0,
              SERVO_MAX_ANGLE,
              SERVO_RR_ZERO_ANGLE,
              SERVO_MAX_STEP_RATE,
              SERVO_MIN_VALUE, SERVO_MAX_ANGLE);
//              49, 191);
    DriveInit(g_drive,
              frontLeftMotor, frontRightMotor,
              rearLeftMotor, rearRightMotor,
              g_frontLeftServo, g_frontRightServo,
              g_rearLeftServo, g_rearRightServo);
    //
    // Initialize Slide subsystem.
    //
    TouchInit(g_slideTouch, slideTouch);
    PIDCtrlInit(g_slidePidCtrl,
                SLIDE_KP,
                SLIDE_KI,
                SLIDE_KD,
                SLIDE_TOLERANCE,
                SLIDE_SETTLING,
                PIDCTRLO_ABS_SETPT);
    PIDMotorInit(g_slideMotor,
                 slideMotor1,
                 slideMotor2,
                 g_slidePidCtrl,
                 0,
                 &g_slideTouch);

    //
    // Initialize Arm subsystem.
    //
    TouchInit(g_armTouch, armTouch);
    PIDCtrlInit(g_armPidCtrl,
                ARM_KP,
                ARM_KI,
                ARM_KD,
                ARM_TOLERANCE,
                ARM_SETTLING,
                PIDCTRLO_ABS_SETPT | PIDCTRLO_NO_OSCILLATE);
    PIDMotorInit(g_armMotor,
                 armMotor,
                 g_armPidCtrl,
                 0,
                 &g_armTouch);

#ifdef _WILLIAM_ENDEFFECTOR
    //
    //Initialize William subsystem
    //
    ServoInit(g_william, williamServo);
    ServoSetAngle(g_william, WILLIAM_MID_POS);
#else
    //
    // Initialize Scoop subsystem
    //
    ServoInit(g_scoop, leftScoopServo, rightScoopServo, SERVOO_SERVO1_REVERSED);
    ServoSetAngle(g_scoop, SCOOP_UP_POS);
#endif
    //
    // Initialize miscellaneous.
    //
    BattInit(g_batt, 5, true);
}   //CommonInit

//
// Other periodic tasks.
//

/**
 *  This function is called periodically to perform tasks common to both
 *  Autonomous and TeleOp modes that require high resolution (e.g. gyro
 *  integration).
 */
void CommonHiFreqTasks()
{
}   //CommonHiFreqTasks

/**
 *  This function is called periodically to perform tasks related to sensors
 *  and inputs that are common to both Autonomous and TeleOp modes.
 */
void CommonInputTasks()
{
    BattTask(g_batt);
    TouchTask(g_slideTouch);
    TouchTask(g_armTouch);
}   //CommonInputTasks

/**
 *  This function is called periodically to perform tasks related to motors
 *  and other outputs that are common to both Autonomous and TeleOp modes.
 */
void CommonOutputTasks()
{
#ifdef _WILLIAM_ENDEFFECTOR
    ServoTask(g_william);
#else
    ServoTask(g_scoop);
#endif
    PIDMotorTask(g_armMotor);
    PIDMotorTask(g_slideMotor);
    DriveTask(g_drive);
#ifdef _DEBUG_SERVO
    ServoTask(g_rearRightServo, 4);
#else
    ServoTask(g_rearRightServo);
#endif
#ifdef _DEBUG_SERVO
    ServoTask(g_rearLeftServo, 3);
#else
    ServoTask(g_rearLeftServo);
#endif
#ifdef _DEBUG_SERVO
    ServoTask(g_frontRightServo, 2);
#else
    ServoTask(g_frontRightServo);
#endif
#ifdef _DEBUG_SERVO
    ServoTask(g_frontLeftServo, 1);
#else
    ServoTask(g_frontLeftServo);
#endif
}   //CommonOutputTasks
