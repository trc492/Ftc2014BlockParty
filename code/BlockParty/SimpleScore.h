void DoSimpleScore(SM &sm, unsigned long delay, int parkOption, int parkDist)
{
    static float startHeading = 0.0;

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
                // Raise arm and optionally wait delay.
                //
                startHeading = PIDCtrlModeGetInput(g_gyroTurnPidCtrl);
                PIDMotorSetTarget(g_armMotor,
                                  ARM_SCORE_POS,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDARM,
                                  3000);
                PIDMotorSetTarget(g_slideMotor,
                                  6.5,
                                  false);
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
                // Drive forward to the basket using sonar.
                //
                PIDDriveSetTarget(g_sonarPidDrive,
                                  0.0,
                                  SONAR_TARGET,
                                  0.0,
                                  0,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE,
                                  2000);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 2:
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

            case SMSTATE_STARTED + 3:
                //
                // Backup a little.
                //
#ifndef _WILLIAM_ENDEFFECTOR
                ServoSetAngle(g_scoop,SCOOP_UP_POS);
#endif
                PIDDriveSetTarget(g_sonarPidDrive,
                                  0.0,
                                  SONAR_TARGET + 4.0,
                                  0.0,
                                  0,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE,
                                  2000);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 4:
                //
                // Turn to clear the pendulum.
                //
                if (parkOption == PARK_OPTION_NONE)
                {
                    //
                    // No-park, we are done.
                    //
                    SMSetState(sm, 9999);   //go to default
                }
                else
                {
                    //
                    // Turn to clear the pendulum and lower arm.
                    //
                    currHeading = PIDCtrlModeGetInput(g_gyroTurnPidCtrl);
                    turnAngle = (parkOption == PARK_OPTION_LEFT)?
                                    startHeading - currHeading - 45.0:
                                    startHeading - currHeading + 45.0;
                    PIDDriveSetTarget(g_turnPidDrive,
                                      0.0,
                                      0.0,
                                      turnAngle,
                                      0,
                                      false,
                                      &sm,
                                      EVTTYPE_PIDDRIVE,
                                      2000);
                    SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                    SMWaitEvents(sm, currState + 1);
                }
                break;

            case SMSTATE_STARTED + 5:
                //
                // Go forward to find the line.
                //
                PIDCtrlSetPowerLimits(g_encoderYPidCtrl, -20.0, 20.0);
                SensorSetEnabled(g_color, true);
                PIDMotorSetTarget(g_slideMotor,
                                  SLIDE_LOWER_LIMIT,
                                  false);
                PIDMotorSetTarget(g_armMotor,
                                  10.0,
                                  false);
                PIDDriveSetTarget(g_yPidDrive,
                                  0.0,
                                  60.0,
                                  0.0,
                                  0,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE,
                                  6000);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 6:
                //
                // Turn towards the ramp.
                //
                SensorSetEnabled(g_color, false);
                PIDCtrlSetPowerLimits(g_encoderYPidCtrl,
                                      DRIVE_MIN_VALUE,
                                      DRIVE_MAX_VALUE);
                currHeading = PIDCtrlModeGetInput(g_gyroTurnPidCtrl);
                turnAngle = (parkOption == PARK_OPTION_LEFT)?
                                startHeading - currHeading + 90.0:
                                startHeading - currHeading - 90.0;
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

            case SMSTATE_STARTED + 7:
                //
                // Go forward up the ramp.
                //
                PIDDriveSetTarget(g_yPidDrive,
                                  0.0,
                                  (parkDist == PARK_DIST_NEAR)? 45.0: 52.0,
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
}   //DoSimpleScore
