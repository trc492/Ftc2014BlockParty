void DoIRScore(SM &sm, unsigned long delay, int irOption, int parkDist)
{
    static float startHeading = 0.0;
    static float startX = 0.0;
    static float xDist = 0.0;

//    PIDCtrlDisplayInfo(1, g_sonarPidCtrl);
//    PIDCtrlDisplayInfo(3, g_irPidCtrl);
//    PIDCtrlDisplayInfo(1, g_encoderXPidCtrl);
//    PIDCtrlDebugInfo(g_sonarPidCtrl);
//    PIDCtrlDebugInfo(g_irPidCtrl);
//    PIDCtrlDebugInfo(g_encoderYPidCtrl);
//    TPrintfLine("Intensity=%5.1f", PIDCtrlModeGetInput(g_colorPidCtrl));
//    PIDCtrlDisplayInfo(1, g_armPidCtrl);
//    PIDCtrlDisplayInfo(3, g_sonarPidCtrl);
//    PIDCtrlDisplayInfo(1, g_encoderYPidCtrl);
//    PIDCtrlDisplayInfo(3, g_gyroTurnPidCtrl);
//    PIDCtrlDisplayInfo(1, g_accelPidCtrl);
//    PIDCtrlDisplayInfo(3, g_colorPidCtrl);
//    nxtDisplayTextLine(1, "color=%d", HTCSreadColor(colorSensor));
//    nxtDisplayTextLine(2, "intensity=%3.0f", PIDCtrlModeGetInput(g_colorPidCtrl));
//    nxtDisplayTextLine(1, "X=%5.1f,Y=%5.1f",
//                       AccelGetXAccel(g_accel),
//                       AccelGetYAccel(g_accel));
//    nxtDisplayTextLine(2, "Z=%5.1f",
//                       AccelGetZAccel(g_accel));

    if (SMIsReady(sm))
    {
        int currState = SMGetState(sm);
        float currHeading;
        float turnAngle;

        switch (currState)
        {
            case SMSTATE_STARTED:
                //
                // Raise the arm and optionally wait delay.
                //
                startHeading = PIDCtrlModeGetInput(g_gyroTurnPidCtrl);
                startX = PIDCtrlModeGetInput(g_encoderXPidCtrl);
                PIDMotorSetTarget(g_armMotor,
                                  ARM_SCORE_POS,
                                  false);
#if 0
                                  &sm,
                                  EVTTYPE_PIDARM,
                                  2000);
#endif
                PIDMotorSetTarget(g_slideMotor,
                                  6.0,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDARM,
                                  2000);
                SMAddWaitEvent(sm, EVTTYPE_PIDARM);
                if (delay != 0)
                {
                    TimerSet(g_timer, delay, &sm, EVTTYPE_TIMER);
                    SMAddWaitEvent(sm, EVTTYPE_TIMER);
                }
                SMWaitEvents(sm, currState + 1, 0, SMO_WAIT_ALL);
                break;

            case SMSTATE_STARTED + 1:
                //
                // Crab sideway to seek IR.
                //
                PIDDriveSetTarget(g_irPidDrive,
                                  IROptionStartRight(irOption)?
                                    IR_TARGET - 0.3: IR_TARGET + 0.3,
                                  0.0,
                                  0.0,
                                  0,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE,
                                  3000);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 2:
                //
                // Drive forward to the basket.
                //
                xDist = abs(PIDCtrlModeGetInput(g_encoderXPidCtrl) -
                            startX);
                nxtDisplayTextLine(1, "xS=%4.1f,xD=%4.1f", startX, xDist);
                PIDDriveSetTarget(g_sonarIrPidDrive,
                                  0.0,
                                  SONAR_TARGET,
                                  IR_TARGET,
                                  SWERVE_DELAY,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE,
                                  3000);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 3:
                //
                // Dump block.
                //
#ifdef _WILLIAM_ENDEFFECTOR
                ServoSetAngle(g_william,WILLIAM_UP_POS);
#else
                ServoSetAngle(g_scoop,SCOOP_DOWN_POS);
#endif
                TimerSet(g_timer, 1000, &sm, EVTTYPE_TIMER);
                SMAddWaitEvent(sm, EVTTYPE_TIMER);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 4:
                //
                // Backup a little.
                //
#ifndef _WILLIAM_ENDEFFECTOR
                ServoSetAngle(g_scoop,SCOOP_UP_POS);
#endif
                PIDDriveSetTarget(g_sonarPidDrive,
                                  0.0,
                                  (xDist > 24.0)?
                                    17:
                                    20 ,
                                  0.0,
                                  0,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE,
                                  1000);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 5:
                if (!IROptionPark(irOption))
                {
                    //
                    // No-park, we are done.
                    //
                    SMSetState(sm, 9999);   //go to default
                }
                else
                {
                    //
                    // Turn to parallel pendulum.
                    //
#ifdef _USE_RADAR
                    if (IROptionStartRight(irOption))
                    {
                        RadarGetDistance(g_leftRadar, -45.0);
                    }
                    else
                    {
                        RadarGetDistance(g_rightRadar, 45.0);
                    }
#endif
                    currHeading = PIDCtrlModeGetInput(g_gyroTurnPidCtrl);
                    turnAngle = IROptionStartRight(irOption)?
                                    startHeading - currHeading - 80.0:
                                    startHeading - currHeading + 80.0;
                    PIDDriveSetTarget(g_turnPidDrive,
                                      0.0,
                                      0.0,
                                      turnAngle,
                                      0,
                                      false,
                                      &sm,
                                      EVTTYPE_PIDDRIVE,
                                      3000);
                    SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                    SMWaitEvents(sm, currState + 1);
                }
                break;

            case SMSTATE_STARTED + 6:
                //
                // Lower the arm and go forward up to clear pendulum.
                //
#if 0
                PIDMotorZeroCalibrate(g_armMotor,
                                      ARM_CAL_POWER,
                                      ARM_STALL_MINPOWER,
                                      ARM_STALL_TIMEOUT);
                PIDMotorZeroCalibrate(g_slideMotor,
                                      SLIDE_CAL_POWER,
                                      SLIDE_STALL_MINPOWER,
                                      SLIDE_STALL_TIMEOUT);
#endif
                PIDMotorSetTarget(g_slideMotor,
                                  SLIDE_LOWER_LIMIT,
                                  false);
                PIDMotorSetTarget(g_armMotor,
                                  10.0,
                                  false);
                if (xDist > IR_PENDULUM_LEN)
                {
                    SMSetState(sm, currState + 1);
                }
                else
                {
#ifdef _USE_RADAR
                    SensorSetEnabled(g_radarSonar, true);
#endif
                    nxtDisplayTextLine(2, "Sonar=%5.1f",
                                       USreadDist(radarSonar)*INCHES_PER_CM);
                    PIDDriveSetTarget(g_yPidDrive,
                                      0.0,
                                      IR_PENDULUM_LEN - xDist,
                                      0.0,
                                      0,
                                      false,
                                      &sm,
                                      EVTTYPE_PIDDRIVE,
                                      3000);
                    SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                    SMWaitEvents(sm, currState + 1);
                }
                break;

            case SMSTATE_STARTED + 7:
                //
                // Turn towards the line.
                //
#ifdef _USE_RADAR
                SensorSetEnabled(g_radarSonar, false);
#endif
                currHeading = PIDCtrlModeGetInput(g_gyroTurnPidCtrl);
                turnAngle = IROptionStartRight(irOption)?
                                startHeading - currHeading - 32.0:
                                startHeading - currHeading + 32.0;
                PIDDriveSetTarget(g_turnPidDrive,
                                  0.0,
                                  0.0,
                                  turnAngle,
                                  0,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE,
                                  5000);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 8:
                //
                // Go forward and look for the white line.
                //
                PIDCtrlSetPowerLimits(g_encoderYPidCtrl, -30.0, 30.0);
                SensorSetEnabled(g_color, true);
                PIDDriveSetTarget(g_yPidDrive,
                                  0.0,
                                  60.0,
                                  0.0,
                                  0,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE,
                                  5000);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 9:
                //
                // Turn towards the ramp.
                //
                SensorSetEnabled(g_color, false);
                PIDCtrlSetPowerLimits(g_encoderYPidCtrl,
                                      DRIVE_MIN_VALUE,
                                      DRIVE_MAX_VALUE);
                currHeading = PIDCtrlModeGetInput(g_gyroTurnPidCtrl);
                turnAngle = IROptionStartRight(irOption)?
                                startHeading - currHeading + 80.0:
                                startHeading - currHeading - 80.0;
                PIDDriveSetTarget(g_turnPidDrive,
                                  0.0,
                                  0.0,
                                  turnAngle,
                                  0,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE,
                                  3000);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 10:
                //
                // Go forward up the ramp.
                //
                PIDDriveSetTarget(g_yPidDrive,
                                  0.0,
                                  (parkDist == PARK_DIST_NEAR)? 48.0: 52.0,
                                  0.0,
                                  0,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE,
                                  3000);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState+1);
                break;

            default:
                //
                // We are done, stop!
                //
                SMStop(sm);
                PlayTone(440, 50);
                break;
        }
    }
}   //DoIRScore
