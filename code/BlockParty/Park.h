void DoPark(SM &sm, unsigned long delay, int parkOption, int parkDist)
{
    if (SMIsReady(sm))
    {
        if (parkOption == PARK_OPTION_NONE)
        {
            SMStop(sm);
        }
        else
        {
            int currState = SMGetState(sm);

            switch (currState)
            {
                case SMSTATE_STARTED:
                    //Drive forward & raise arm to go above lip
                    PIDMotorSetTarget(g_armMotor,
                                      10.0,
                                      false);
                    PIDDriveSetTarget(g_yPidDrive,
                                      0.0,
                                      30.0,
                                      0.0,
                                      0,
                                      false,
                                      &sm,
                                      EVTTYPE_PIDDRIVE,
                                      5000);
                    SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                    SMWaitEvents(sm, currState + 1);
                    break;

                case SMSTATE_STARTED + 1:
                    PIDDriveSetTarget(g_turnPidDrive,
                                      0.0,
                                      0.0,
                                      (parkOption == PARK_OPTION_LEFT)?
                                        40.0: -40.0,
                                      0,
                                      false,
                                      &sm,
                                      EVTTYPE_PIDDRIVE,
                                      3000);
                    SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                    SMWaitEvents(sm, currState + 1);
                    break;

                case SMSTATE_STARTED + 2:
                    PIDDriveSetTarget(g_yPidDrive,
                                      0.0,
                                      (parkDist == PARK_DIST_NEAR)? 30.0: 36.0,
                                      0.0,
                                      0,
                                      false,
                                      &sm,
                                      EVTTYPE_PIDDRIVE,
                                      5000);
                    SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                    SMWaitEvents(sm, currState + 1);
                    break;

                default:
                    SMStop(sm);
                    PlayTone(440, 50);
                    break;
            }
        }
    }
}   //DoPark
