#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="AutoCode.h" />
///
/// <summary>
///     This module contains the Autonomous-only functions.
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
#include "..\ftclib\kalman.h"
#include "..\ftclib\gyro.h"
#ifdef _USE_ACCEL
#include "..\ftclib\accel.h"
#endif
#include "..\ftclib\irseeker.h"
#include "..\ftclib\sensor.h"
#include "..\ftclib\timer.h"
#include "..\ftclib\menu.h"
#include "..\ftclib\piddrive.h"
#ifdef _USE_RADAR
#include "..\ftclib\radar.h"
#endif

//
// Constants.
//

// Event types.
#define EVTTYPE_TIMER           (EVTTYPE_NONE + 1)
#define EVTTYPE_PIDDRIVE        (EVTTYPE_NONE + 2)
#define EVTTYPE_PIDSLIDE        (EVTTYPE_NONE + 3)
#define EVTTYPE_PIDARM          (EVTTYPE_NONE + 4)

//
// Global data.
//

// Inputs
GYRO            g_gyro;
#ifdef _USE_ACCEL
ACCEL           g_accel;
#endif
IRSEEKER        g_leftIR;
IRSEEKER        g_rightIR;
SENSOR          g_color;
#ifdef _USE_RADAR
SENSOR          g_radarSonar;
SERVO           g_leftRadarServo;
SERVO           g_rightRadarServo;
RADAR           g_leftRadar;
RADAR           g_rightRadar;
#endif

// PID controls
PIDCTRL         g_encoderXPidCtrl;
PIDCTRL         g_encoderYPidCtrl;
PIDCTRL         g_gyroRotPidCtrl;
PIDCTRL         g_gyroTurnPidCtrl;
PIDCTRL         g_irPidCtrl;
PIDCTRL         g_sonarPidCtrl;
PIDCTRL         g_colorPidCtrl;
#ifdef _USE_ACCEL
PIDCTRL         g_accelPidCtrl;
#endif

PIDDRIVE        g_xPidDrive;
PIDDRIVE        g_yPidDrive;
PIDDRIVE        g_rotPidDrive;
PIDDRIVE        g_turnPidDrive;
PIDDRIVE        g_irPidDrive;
PIDDRIVE        g_sonarPidDrive;
PIDDRIVE        g_sonarIrPidDrive;
#ifdef _USE_ACCEL
PIDDRIVE        g_colorPidDrive;
#endif

// Miscellaneous
TIMER           g_timer;
SM              g_autoSM;

// Menus
MENU            g_strategyMenu;
MENU            g_autoDelayMenu;
MENU            g_defenseDistMenu;
MENU            g_parkOptionMenu;
MENU            g_irOptionMenu;
MENU            g_parkDistMenu;

int             g_strategy =  STRATEGY_IR_SCORE;
unsigned long   g_autoDelay = 0;
float           g_defenseDist = 0.0;
int             g_parkOption = PARK_OPTION_NONE;
int             g_irOption = IR_OPTION_START_RIGHT | IR_OPTION_PARK;
int             g_parkDist = PARK_DIST_NONE;

//
// Include autonomous strategies.
//
#include "Defense.h"
#include "SimpleScore.h"
#include "IRScore.h"
#include "Park.h"
#include "TestMode.h"

//
// Callback functions.
//
float PIDCtrlModeGetInput(PIDCTRL &pidCtrl)
{
    float inputValue = 0.0;

    if (PIDCtrlCheck(pidCtrl, g_encoderXPidCtrl))
    {
        inputValue = DriveGetXPos(g_drive)/CLICKS_PER_XUNITS;
    }
    else if (PIDCtrlCheck(pidCtrl, g_encoderYPidCtrl))
    {
        inputValue = DriveGetYPos(g_drive)/CLICKS_PER_YUNITS;
    }
    else if (PIDCtrlCheck(pidCtrl, g_gyroRotPidCtrl) ||
             PIDCtrlCheck(pidCtrl, g_gyroTurnPidCtrl))
    {
        inputValue = GyroGetHeading(g_gyro);
    }
    else if (PIDCtrlCheck(pidCtrl, g_sonarPidCtrl))
    {
        inputValue = USreadDist(sonarSensor)*INCHES_PER_CM;
    }
    else if (PIDCtrlCheck(pidCtrl, g_irPidCtrl))
    {
        inputValue = IRSeekerGetACDir(g_leftIR) +
                     IRSeekerGetACDir(g_rightIR);
    }
    else if (PIDCtrlCheck(pidCtrl, g_colorPidCtrl))
    {
        int color = HTCSreadColor(colorSensor);
        int red, green, blue;
        HTCSreadRGB(colorSensor, red, green, blue);
        inputValue = ((color == COLOR_WHITE) || (color == COLOR_BLACK))?
                        (red + green + blue)/3.0: 0.0;
    }
#ifdef _USE_ACCEL
    else if (PIDCtrlCheck(pidCtrl, g_accelPidCtrl))
    {
        inputValue = AccelGetZAccel(g_accel);
    }
#endif

    return inputValue;
}   //PIDCtrlModeGetInput

float SensorGetValue(SENSOR &sensor)
{
    float value = 0.0;

    if (&sensor == &g_color)
    {
        value = PIDCtrlModeGetInput(g_colorPidCtrl);
    }
#ifdef _USE_RADAR
    if (&sensor == &g_radarSonar)
    {
        value = USreadDist(radarSonar)*INCHES_PER_CM;
    }
#endif

    return value;
}   //SensorGetValue

void SensorEvent(SENSOR &sensor, int zone)
{
    if (&sensor == &g_color)
    {
        if (zone == SENSORZONE_HIGH)
        {
            if (PIDDriveIsEnabled(g_yPidDrive))
            {
                PIDDriveAbort(g_yPidDrive);
            }
            else if (PIDDriveIsEnabled(g_turnPidDrive))
            {
                PIDDriveAbort(g_turnPidDrive);
            }
        }
    }
#ifdef _USE_RADAR
    else if (&sensor == &g_radarSonar)
    {
        if (zone == SENSORZONE_LOW)
        {
            nxtDisplayTextLine(4, "Sonar Aborted.");
            if (PIDDriveIsEnabled(g_yPidDrive))
            {
                PIDDriveAbort(g_yPidDrive);
            }
            else if (PIDDriveIsEnabled(g_turnPidDrive))
            {
                PIDDriveAbort(g_turnPidDrive);
            }
        }
    }
#endif
}   //SensorEvent

//
// Main functions.
//

/**
 *  This function is called once globally to do Autonomous specific
 *  initializations.
 */
void AutoInit()
{
    // Inputs
    GyroInit(g_gyro, gyroSensor);
#ifdef _USE_ACCEL
    AccelInit(g_accel, accelSensor, ACCELO_FILTER);
#endif
    IRSeekerInit( g_leftIR, leftIR);
    IRSeekerInit(g_rightIR,rightIR);
    SensorInit(g_color, colorSensor,
               COLOR_WHITE_THRESHOLD, COLOR_WHITE_THRESHOLD);
#ifdef _USE_RADAR
    SensorInit(g_radarSonar, radarSonar, 10.0, 10.0);
    ServoInit(g_leftRadarServo, leftRadarServo, 0, 180.0, 90.0);
    ServoInit(g_rightRadarServo, rightRadarServo, 0, 180.0, 90.0);
    RadarInit(g_leftRadar, radarSonar, &g_leftRadarServo, 500);
    RadarInit(g_rightRadar, radarSonar, &g_rightRadarServo, 500);
    RadarAddSamplePoint(g_leftRadar, -90.0);
    RadarAddSamplePoint(g_leftRadar, -45.0);
    RadarAddSamplePoint(g_leftRadar, 0.0);
    RadarAddSamplePoint(g_leftRadar, 45.0);
    RadarAddSamplePoint(g_leftRadar, 90.0);
    RadarAddSamplePoint(g_rightRadar, -90.0);
    RadarAddSamplePoint(g_rightRadar, -45.0);
    RadarAddSamplePoint(g_rightRadar, 0.0);
    RadarAddSamplePoint(g_rightRadar, 45.0);
    RadarAddSamplePoint(g_rightRadar, 90.0);
#endif

    //
    // Perform slide and arm zero calibration:
    // To prevent the motor from being fried because of the touch sensor
    // failed to engage, the following code will stop the motor as soon
    // as the touch sensor is engaged or if the elevator is stuck for
    // more than the timeout period. There are a number of reasons the
    // touch sensor failed to engage. It could be a mechanical problem
    // or the touch sensor is on a MUX and we forgot to turn the MUX power
    // ON.
    //
    PIDMotorZeroCalibrate(g_armMotor,
                          ARM_CAL_POWER,
                          ARM_STALL_MINPOWER,
                          ARM_STALL_TIMEOUT);
    PIDMotorZeroCalibrate(g_slideMotor,
                          SLIDE_CAL_POWER,
                          SLIDE_STALL_MINPOWER,
                          SLIDE_STALL_TIMEOUT);

    //Drive subsystem
    PIDCtrlInit(g_encoderXPidCtrl,
                ENCX_KP, ENCX_KI, ENCX_KD,
                ENCX_TOLERANCE, ENCX_SETTLING);
    PIDCtrlSetPowerLimits(g_encoderXPidCtrl,
                          DRIVE_MIN_VALUE,
                          DRIVE_MAX_VALUE);
    PIDCtrlInit(g_encoderYPidCtrl,
                ENCY_KP, ENCY_KI, ENCY_KD,
                ENCY_TOLERANCE, ENCY_SETTLING);
    PIDCtrlSetPowerLimits(g_encoderYPidCtrl,
                          DRIVE_MIN_VALUE,
                          DRIVE_MAX_VALUE);
    PIDCtrlInit(g_gyroRotPidCtrl,
                GYROROT_KP, GYROROT_KI, GYROROT_KD,
                GYROROT_TOLERANCE, GYROROT_SETTLING);
    PIDCtrlInit(g_gyroTurnPidCtrl,
                GYROTURN_KP, GYROTURN_KI, GYROTURN_KD,
                GYROTURN_TOLERANCE, GYROTURN_SETTLING);
    PIDCtrlSetPowerLimits(g_gyroTurnPidCtrl,
                          TURN_MIN_VALUE, TURN_MAX_VALUE);
    PIDCtrlInit(g_irPidCtrl,
                IR_KP, IR_KI, IR_KD,
                IR_TOLERANCE, IR_SETTLING, PIDCTRLO_ABS_SETPT | PIDCTRLO_INVERSE);
    PIDCtrlInit(g_sonarPidCtrl,
                SONAR_KP, SONAR_KI, SONAR_KD,
                SONAR_TOLERANCE, SONAR_SETTLING, PIDCTRLO_ABS_SETPT | PIDCTRLO_INVERSE);
    PIDCtrlInit(g_colorPidCtrl,
                COLOR_KP, COLOR_KI, COLOR_KD,
                COLOR_TOLERANCE, COLOR_SETTLING, PIDCTRLO_ABS_SETPT);
#ifdef _USE_ACCEL
    PIDCtrlInit(g_accelPidCtrl,
                ACCEL_KP, ACCEL_KI, ACCEL_KD,
                ACCEL_TOLERANCE, ACCEL_SETTLING, PIDCTRLO_ABS_SETPT | PIDCTRLO_INVERSE);
#endif

    PIDDriveInit(g_xPidDrive,
                 g_drive,
                 &g_encoderXPidCtrl,
                 NULL,
                 &g_gyroTurnPidCtrl,
                 PIDDRIVEO_DIFF_ROT);
    PIDDriveInit(g_yPidDrive,
                 g_drive,
                 NULL,
                 &g_encoderYPidCtrl,
                 &g_gyroTurnPidCtrl,
                 PIDDRIVEO_DIFF_ROT);
    PIDDriveInit(g_rotPidDrive,
                 g_drive,
                 NULL,
                 NULL,
                 &g_gyroRotPidCtrl);
    PIDDriveInit(g_turnPidDrive,
                 g_drive,
                 NULL,
                 NULL,
                 &g_gyroTurnPidCtrl,
                 PIDDRIVEO_DIFF_ROT);
    PIDDriveInit(g_irPidDrive,
                 g_drive,
                 &g_irPidCtrl,
                 NULL,
                 NULL,
                 PIDDRIVEO_DIFF_ROT);
    PIDDriveInit(g_sonarPidDrive,
                 g_drive,
                 NULL,
                 &g_sonarPidCtrl,
                 &g_gyroTurnPidCtrl,
                 PIDDRIVEO_DIFF_ROT);
    PIDDriveInit(g_sonarIrPidDrive,
                 g_drive,
                 NULL,
                 &g_sonarPidCtrl,
                 &g_irPidCtrl,
                 PIDDRIVEO_DIFF_ROT);
#ifdef _USE_ACCEL
    PIDDriveInit(g_colorPidDrive,
                 g_drive,
                 NULL,
                 &g_accelPidCtrl,
                 &g_colorPidCtrl,
                 PIDDRIVEO_DIFF_ROT);
#endif

    TimerInit(g_timer);
    SMInit(g_autoSM);

    //
    // Initialize menus.
    //

    // Strategy menu.
    MenuInit(g_strategyMenu, "Strategies:");
    MenuAddChoice(g_strategyMenu, "No Auto", (float)STRATEGY_NOAUTO);
    MenuAddChoice(g_strategyMenu, "Defense", (float)STRATEGY_DEFENSE);
    MenuAddChoice(g_strategyMenu, "Simple Score", (float)STRATEGY_SIMPLE_SCORE);
    MenuAddChoice(g_strategyMenu, "IR Score", (float)STRATEGY_IR_SCORE);
    MenuAddChoice(g_strategyMenu, "Park Only", (float)STRATEGY_PARK_ONLY);
#ifdef _ENABLE_TESTMODES
    MenuAddChoice(g_strategyMenu, "Test Mode", (float)STRATEGY_TEST_MODE);
#endif

    // Autonomous delay menu.
    MenuInit(g_autoDelayMenu, "Autonomous Delay:");
    MenuAddChoice(g_autoDelayMenu, "None", 0.0);
    MenuAddChoice(g_autoDelayMenu, "1 sec", 1000.0);
    MenuAddChoice(g_autoDelayMenu, "2 sec", 2000.0);
    MenuAddChoice(g_autoDelayMenu, "4 sec", 4000.0);
    MenuAddChoice(g_autoDelayMenu, "6 sec", 6000.0);
    MenuAddChoice(g_autoDelayMenu, "8 sec", 8000.0);
    MenuAddChoice(g_autoDelayMenu, "10 sec", 10000.0);
    MenuAddChoice(g_autoDelayMenu, "12 sec", 12000.0);

    // Defense distance menu.
    MenuInit(g_defenseDistMenu, "Defense Distance:");
    MenuAddChoice(g_defenseDistMenu, "4 ft", 48.0);
    MenuAddChoice(g_defenseDistMenu, "6 ft", 72.0);
    MenuAddChoice(g_defenseDistMenu, "8 ft", 96.0);

    // Park options menu.
    MenuInit(g_parkOptionMenu, "Parking options:");
    MenuAddChoice(g_parkOptionMenu, "No park"   , PARK_OPTION_NONE );
    MenuAddChoice(g_parkOptionMenu, "Left side" , PARK_OPTION_LEFT );
    MenuAddChoice(g_parkOptionMenu, "Right side", PARK_OPTION_RIGHT);

    // IR menu
    MenuInit(g_irOptionMenu, "IR score options");
    MenuAddChoice(g_irOptionMenu, "Start L,No Park", IR_OPTION_START_LEFT);
    MenuAddChoice(g_irOptionMenu, "Start L,Park",    IR_OPTION_START_LEFT | IR_OPTION_PARK);
    MenuAddChoice(g_irOptionMenu, "Start R,No Park", IR_OPTION_START_RIGHT);
    MenuAddChoice(g_irOptionMenu, "Start R,Park",    IR_OPTION_START_RIGHT | IR_OPTION_PARK);

    // Park distance menu
    MenuInit(g_parkDistMenu, "Park Distance:");
    MenuAddChoice(g_parkDistMenu, "Near", PARK_DIST_NEAR);
    MenuAddChoice(g_parkDistMenu, "Far", PARK_DIST_FAR);

    g_strategy  = (int)MenuGetChoice(g_strategyMenu);

#ifdef _ENABLE_TESTMODES
    if (g_strategy == STRATEGY_TEST_MODE)
    {
        TestModeInit();
    }
    else
#endif
    if (g_strategy != STRATEGY_NOAUTO)
    {
        g_autoDelay = (unsigned long)MenuGetChoice(g_autoDelayMenu);
        if (g_strategy == STRATEGY_DEFENSE)
        {
            g_defenseDist = MenuGetChoice(g_defenseDistMenu);
        }
        else
        {
            if (g_strategy == STRATEGY_IR_SCORE)
            {
                g_irOption = MenuGetChoice(g_irOptionMenu);
                //
                // Prep the wheels for crabbing.
                //
                float angle = (g_irOption & IR_OPTION_START_LEFT)? 90.0 : -90.0;
                DriveSwerveSetAngles(g_drive, angle, angle, angle, angle);
                if (IROptionPark(g_irOption))
                {
                    g_parkDist = MenuGetChoice(g_parkDistMenu);
                }
            }
            else
            {
                //
                // Simple Score or Park-only.
                //
                g_parkOption = MenuGetChoice(g_parkOptionMenu);
                if (g_parkOption != PARK_OPTION_NONE)
                {
                    g_parkDist = MenuGetChoice(g_parkDistMenu);
                }
            }
        }
    }

    nxtDisplayCenteredTextLine(0, "%s",
#ifdef _ENABLE_TESTMODES
                               (g_strategy == STRATEGY_TEST_MODE)?
                                    MenuGetChoiceText(m_testModeMenu):
                                    MenuGetChoiceText(g_strategyMenu));
#else
                               MenuGetChoiceText(g_strategyMenu));
#endif
    nxtDisplayCenteredTextLine(3, "Wait to Start...");
}   //AutoInit


/**
 *  This function is called periodically to perform tasks specific to
 *  Autonomous mode that require high resolution (e.g. gyro integration).
 */
void AutoHiFreqTasks()
{
    GyroTask(g_gyro);
#ifdef _USE_ACCEL
    AccelTask(g_accel);
#endif
    TimerTask(g_timer);
#ifdef _USE_RADAR
    SensorTask(g_radarSonar);
#endif
    SensorTask(g_color);
}   //AutoHiFreqTasks

/**
 *  This function is called periodically to perform tasks related to sensors
 *  and inputs that are specific to Autonomous mode.
 */
void AutoInputTasks()
{
    IRSeekerTask( g_leftIR);
    IRSeekerTask(g_rightIR);
}   //AutoInputTasks

/**
 *  This function is called periodically to perform tasks related to motors
 *  and other outputs that are specific to Autonomous mode.
 */
void AutoOutputTasks()
{
#ifdef _USE_RADAR
    RadarTask(g_leftRadar);
    RadarTask(g_rightRadar);
    ServoTask(g_leftRadarServo);
    ServoTask(g_rightRadarServo);
#endif
#ifdef _USE_ACCEL
    PIDDriveTask(g_colorPidDrive);
#endif
    PIDDriveTask(g_sonarIrPidDrive);
    PIDDriveTask(g_sonarPidDrive);
    PIDDriveTask(g_irPidDrive);
    PIDDriveTask(g_turnPidDrive);
    PIDDriveTask(g_rotPidDrive);
    PIDDriveTask(g_yPidDrive);
    PIDDriveTask(g_xPidDrive);
}   //AutoOutputTasks

/**
 *  This function is called before Autonomous mode starts.
 */
void
AutoStart()
{
    SMStart(g_autoSM);
}   //AutoStart

/**
 *  This function is called after Autonomous mode ends.
 */
void
AutoStop()
{
    SMStop(g_autoSM);
}   //AutoStop

/**
 *  This function is called periodically to perform the Autonomous tasks.
 *
 *  @param time Specifies the running time since Autonomous started.
 */
void AutoTasks(
    float time
    )
{
    nxtDisplayTextLine(0, "Auto: %d [%5.1f]",
                       SMGetState(g_autoSM) - SMSTATE_STARTED, time);

    switch (g_strategy)
    {
        case STRATEGY_NOAUTO:
            SMStop(g_autoSM);
            break;

        case STRATEGY_DEFENSE:
            DoDefense(g_autoSM, g_autoDelay, g_defenseDist);
            break;

        case STRATEGY_SIMPLE_SCORE:
            DoSimpleScore(g_autoSM, g_autoDelay, g_parkOption, g_parkDist);
            break;

        case STRATEGY_IR_SCORE:
            DoIRScore(g_autoSM, g_autoDelay, g_irOption, g_parkDist);
            break;

        case STRATEGY_PARK_ONLY:
            DoPark(g_autoSM, g_autoDelay, g_parkOption, g_parkDist);
            break;

#ifdef _ENABLE_TESTMODES
        case STRATEGY_TEST_MODE:
            DoTestMode();
            break;
#endif

        default:
            TErr(("Invalid strategy %d!", g_strategy));
            break;
    }
}   //AutoTasks
