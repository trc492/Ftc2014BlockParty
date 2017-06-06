#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="TestMode.h" />
///
/// <summary>
///     This module contains the test mode code.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifdef _ENABLE_TESTMODES

//
// Constants.
//
#define STRATEGY_TEST_MODE      100

#define TESTMODE_TIMED_DRIVE    0
#define TESTMODE_XDRIVE_4FT     1
#define TESTMODE_YDRIVE_4FT     2
#define TESTMODE_ROT_360        3
#define TESTMODE_TURN_360       4
#define TESTMODE_TEST_IR        5
#define TESTMODE_TEST_IRDRIVE   6
#define TESTMODE_TEST_SONAR     7
#define TESTMODE_TEST_COLOR     8
#define TESTMODE_TEST_RADAR     9

//
// Local data.
//
TIMER   m_testModeTimer;
SM      m_testModeSM;

MENU    m_testModeMenu;
int     m_testMode = TESTMODE_TIMED_DRIVE;

/**
 *  This function initializes the Test Mode menu.
 */
void
TestModeInit(
    void
    )
{
    TimerInit(m_testModeTimer);
    SMInit(m_testModeSM);

    //
    // Initialize TestMode menu.
    //
    MenuInit(m_testModeMenu, "Test modes:");
    MenuAddChoice(m_testModeMenu, "Timed Drive", (float)TESTMODE_TIMED_DRIVE);
    MenuAddChoice(m_testModeMenu, "X Drive 4ft", (float)TESTMODE_XDRIVE_4FT);
    MenuAddChoice(m_testModeMenu, "Y Drive 4ft", (float)TESTMODE_YDRIVE_4FT);
    MenuAddChoice(m_testModeMenu, "Rotate 360", (float)TESTMODE_ROT_360);
    MenuAddChoice(m_testModeMenu, "Turn 360", (float)TESTMODE_TURN_360);
    MenuAddChoice(m_testModeMenu, "Test IR Sensors", (float)TESTMODE_TEST_IR);
    MenuAddChoice(m_testModeMenu, "Test IR Drive", (float)TESTMODE_TEST_IRDRIVE);
    MenuAddChoice(m_testModeMenu, "Test Sonar", (float)TESTMODE_TEST_SONAR);
    MenuAddChoice(m_testModeMenu, "Test Color", (float)TESTMODE_TEST_COLOR);
#ifdef _USE_RADAR
    MenuAddChoice(m_testModeMenu, "Test Radar", (float)TESTMODE_TEST_RADAR);
#endif

    m_testMode = (int)MenuGetChoice(m_testModeMenu);
    SMStart(m_testModeSM);
}   //TestModeInit

/**
 *  This function drives the robot forward for a fixed period of time.
 *
 *  @param sm Specifies the state machine.
 */
void
DoTimedDrive(
    SM &sm
    )
{
    nxtDisplayTextLine(1, "FL=%d,FR=%d",
                       nMotorEncoder[frontLeftMotor], nMotorEncoder[frontRightMotor]);
    nxtDisplayTextLine(2, "RL=%d,RR=%d",
                       nMotorEncoder[rearLeftMotor], nMotorEncoder[rearRightMotor]);
    nxtDisplayTextLine(3, "Enc=%f",
                       (nMotorEncoder[frontLeftMotor] +
                        nMotorEncoder[frontRightMotor] +
                        nMotorEncoder[rearLeftMotor] +
                        nMotorEncoder[rearRightMotor])/4.0);
    if (SMIsReady(sm))
    {
        int currState = SMGetState(sm);

        switch (currState)
        {
            case SMSTATE_STARTED:
                //
                // Drive forward at 50% power for 3.5 seconds.
                //
                DriveSwerve(g_drive, 0, 50, 0);
                TimerSet(m_testModeTimer, 3000, &sm, EVTTYPE_TIMER);
                SMAddWaitEvent(sm, EVTTYPE_TIMER);
                SMWaitEvents(sm, currState + 1);
                break;

            default:
                DriveSwerve(g_drive, 0, 0, 0);
                SMStop(sm);
                PlayTone(440, 50);
                break;
        }
    }

    if (SMIsEnabled(sm))
    {
        PIDCtrlDebugInfo(g_encoderYPidCtrl);
        PIDCtrlDebugInfo(g_gyroTurnPidCtrl);
    }
}   //DoTimedDrive

/**
 *  This function drives the robot sideway for 4 feet.
 *
 *  @param sm Specifies the state machine.
 */
void
DoXDrive4ft(
    SM &sm
    )
{
    PIDCtrlDisplayInfo(1, g_encoderXPidCtrl);
    PIDCtrlDisplayInfo(3, g_gyroTurnPidCtrl);
    if (SMIsReady(sm))
    {
        int currState = SMGetState(sm);

    	switch (currState)
    	{
    	    case SMSTATE_STARTED:
    	        //
    	        // Drive right for 4 ft.
    	        //
    	        PIDDriveSetTarget(g_xPidDrive,
    	                          48.0, 0.0, 0.0,
    	                          0,
    	                          false,
    	                          &sm,
    	                          EVTTYPE_PIDDRIVE);
    	        SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
    	        SMWaitEvents(sm, currState + 1);
    	        break;

    	    default:
    	        SMStop(sm);
    	        PlayTone(440, 50);
    	        break;
    	}
    }

    if (SMIsEnabled(sm))
    {
        PIDCtrlDebugInfo(g_encoderXPidCtrl);
        PIDCtrlDebugInfo(g_gyroTurnPidCtrl);
    }
}   //DoXDrive4ft

/**
 *  This function drives the robot forward for 4 feet.
 *
 *  @param sm Specifies the state machine.
 */
void
DoYDrive4ft(
    SM &sm
    )
{
    PIDCtrlDisplayInfo(1, g_encoderYPidCtrl);
    PIDCtrlDisplayInfo(3, g_gyroTurnPidCtrl);
    if (SMIsReady(sm))
    {
        int currState = SMGetState(sm);

    	switch (currState)
    	{
    	    case SMSTATE_STARTED:
    	        //
    	        // Drive forward for 4 ft.
    	        //
    	        PIDDriveSetTarget(g_yPidDrive,
    	                          0.0, 48.0, 0.0,
    	                          0,
    	                          false,
    	                          &sm,
    	                          EVTTYPE_PIDDRIVE);
    	        SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
    	        SMWaitEvents(sm, currState + 1);
    	        break;

    	    default:
    	        SMStop(sm);
    	        PlayTone(440, 50);
    	        break;
    	}
    }

    if (SMIsEnabled(sm))
    {
        PIDCtrlDebugInfo(g_encoderYPidCtrl);
        PIDCtrlDebugInfo(g_gyroTurnPidCtrl);
    }
}   //DoYDrive4ft

/**
 *  This function rotate the robot clockwise for 360 degrees.
 *
 *  @param sm Specifies the state machine.
 */
void
DoRotate360(
    SM &sm
    )
{
    PIDCtrlDisplayInfo(1, g_gyroRotPidCtrl);
    if (SMIsReady(sm))
    {
        int currState = SMGetState(sm);

        switch (currState)
        {
            case SMSTATE_STARTED:
                //
                // Turn clockwise 90 degrees.
                //
                PIDDriveSetTarget(g_rotPidDrive,
                                  0.0, 0.0, 360.0,
                                  0,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

            default:
                SMStop(sm);
                PlayTone(440, 50);
                break;
        }
    }

    if (SMIsEnabled(sm))
    {
        PIDCtrlDebugInfo(g_gyroRotPidCtrl);
    }
}   //DoRotate360

/**
 *  This function turn the robot clockwise for 360 degrees by doing
 *  wheel differential.
 *
 *  @param sm Specifies the state machine.
 */
void
DoTurn360(
    SM &sm
    )
{
    PIDCtrlDisplayInfo(1, g_gyroTurnPidCtrl);
    if (SMIsReady(sm))
    {
        int currState = SMGetState(sm);

        switch (currState)
        {
            case SMSTATE_STARTED:
                //
                // Turn clockwise 90 degrees.
                //
                PIDDriveSetTarget(g_turnPidDrive,
                                  0.0, 0.0, 360.0,
                                  0,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

            default:
                SMStop(sm);
                PlayTone(440, 50);
                break;
        }
    }

    if (SMIsEnabled(sm))
    {
        PIDCtrlDebugInfo(g_gyroTurnPidCtrl);
    }
}   //DoTurn360

/**
 *  This function tests the IR sensors.
 *
 *  @param sm Specifies the state machine.
 */
void
DoTestIRSensors(
    SM &sm
    )
{
    nxtDisplayTextLine(1, "irL=%4.1f,irR=%4.1f",
                       IRSeekerGetACDir(g_leftIR),
                       IRSeekerGetACDir(g_rightIR));
    if (SMIsReady(sm))
    {
        int currState = SMGetState(sm);

        switch (currState)
        {
            case SMSTATE_STARTED:
                PIDMotorSetTarget(g_armMotor,
                                  60.0,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDARM);
                SMAddWaitEvent(sm, EVTTYPE_PIDARM);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 1:
                break;

            default:
                SMStop(sm);
                PlayTone(440, 50);
                break;
        }
    }
}   //DoTestIRSensors

/**
 *  This function tests the IR drive.
 *
 *  @param sm Specifies the state machine.
 */
void
DoTestIRDrive(
    SM &sm
    )
{
    PIDCtrlDisplayInfo(1, g_irPidCtrl);
    PIDCtrlDisplayInfo(3, g_sonarPidCtrl);
    if (SMIsReady(sm))
    {
        int currState = SMGetState(sm);

        switch (currState)
        {
            case SMSTATE_STARTED:
                PIDMotorSetTarget(g_armMotor,
                                  60.0,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDARM);
                SMAddWaitEvent(sm, EVTTYPE_PIDARM);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 1:
                PIDDriveSetTarget(g_irPidDrive,
                                  IR_TARGET,
                                  0.0,
                                  0.0,
                                  0,
                                  true);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

            default:
                SMStop(sm);
                PlayTone(440, 50);
                break;
        }
    }

    if (SMIsEnabled(sm))
    {
        PIDCtrlDebugInfo(g_irPidCtrl);
        PIDCtrlDebugInfo(g_sonarPidCtrl);
    }
}   //DoTestIRDrive

/**
 *  This function tests the sonar drive.
 *
 *  @param sm Specifies the state machine.
 */
void
DoTestSonar(
    SM &sm
    )
{
    PIDCtrlDisplayInfo(1, g_sonarPidCtrl);
    if (SMIsReady(sm))
    {
        int currState = SMGetState(sm);

        switch (currState)
        {
            case SMSTATE_STARTED:
                PIDMotorSetTarget(g_armMotor,
                                  60.0,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDARM);
                SMAddWaitEvent(sm, EVTTYPE_PIDARM);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 1:
                PIDDriveSetTarget(g_sonarPidDrive,
                                  0.0,
                                  24.0,
                                  0.0,
                                  0,
                                  true);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

            default:
                SMStop(sm);
                PlayTone(440, 50);
                break;
        }
    }
}   //DoTestSonar

#ifdef _USE_RADAR
/**
 *  This function tests the radar.
 *
 *  @param sm Specifies the state machine.
 */
void
DoTestRadar(
    SM &sm
    )
{
    RadarDisplayInfo(1, g_leftRadar);
    RadarDisplayInfo(3, g_rightRadar);
    if (SMIsReady(sm))
    {
        int currState = SMGetState(sm);

        switch (currState)
        {
            case SMSTATE_STARTED:
                RadarEnableScan(g_leftRadar, true);
                RadarEnableScan(g_rightRadar, true);
                SMSetState(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 1:
                break;

            default:
                RadarEnableScan(g_leftRadar, false);
                RadarEnableScan(g_rightRadar, false);
                SMStop(sm);
                PlayTone(440, 50);
                break;
        }
    }
}   //DoTestRadar
#endif

/**
 *  This function tests the color sensor.
 *
 *  @param sm Specifies the state machine.
 */
void
DoTestColor(
    SM &sm
    )
{
    int color = HTCSreadColor(colorSensor);
    int red, green, blue;

    HTCSreadRGB(colorSensor, red, green, blue);
    nxtDisplayTextLine(1, "CS=%d,I=%d", color, (red + green + blue)/3);
    nxtDisplayTextLine(2, "Red=%d", red);
    nxtDisplayTextLine(3, "Green=%d", green);
    nxtDisplayTextLine(4, "Blue=%d", blue);
}   //DoTestColor

/**
 *  This function executes the test mode routine.
 */
void
DoTestMode(
    void
    )
{
    switch (m_testMode)
    {
        case TESTMODE_TIMED_DRIVE:
            DoTimedDrive(m_testModeSM);
            break;

        case TESTMODE_XDRIVE_4FT:
            DoXDrive4ft(m_testModeSM);
            break;

        case TESTMODE_YDRIVE_4FT:
            DoYDrive4ft(m_testModeSM);
            break;

        case TESTMODE_ROT_360:
            DoRotate360(m_testModeSM);
            break;

        case TESTMODE_TURN_360:
            DoTurn360(m_testModeSM);
            break;

        case TESTMODE_TEST_IR:
            DoTestIRSensors(m_testModeSM);
            break;

        case TESTMODE_TEST_IRDRIVE:
            DoTestIRDrive(m_testModeSM);
            break;

        case TESTMODE_TEST_SONAR:
            DoTestSonar(m_testModeSM);
            break;

        case TESTMODE_TEST_COLOR:
            DoTestColor(m_testModeSM);
            break;

#ifdef _USE_RADAR
        case TESTMODE_TEST_RADAR:
            DoTestRadar(m_testModeSM);
            break;
#endif

        default:
            TErr(("Invalid test mode %d!", m_TestMode));
            break;
    }

    TimerTask(m_testModeTimer);
}   //DoTestMode

#endif  //ifdef _ENABLE_TESTMODES
