#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="Defense.h" />
///
/// <summary>
///     This module contains the implementation of the Defense strategy.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

/**
 *  This function drives the robot to block the opponent from doing their
 *  autonomous.
 *
 *  @param sm Specifies the state machine.
 *  @param delay Specifies the delay if any.
 *  @param distance Specifies the distance to travel.
 */
void DoDefense(SM &sm, unsigned long delay, float distance)
{
    PIDCtrlDisplayInfo(1, g_encoderYPidCtrl);
    PIDCtrlDisplayInfo(3, g_gyroTurnPidCtrl);
    if (SMIsReady(sm))
    {
        int currState = SMGetState(sm);

        switch (currState)
        {
            case SMSTATE_STARTED:
                if (delay != 0)
                {
                    TimerSet(g_timer, delay, &sm, EVTTYPE_TIMER);
                    SMAddWaitEvent(sm, EVTTYPE_TIMER);
                    SMWaitEvents(sm, currState + 1);
                }
                else
                {
                    SMSetState(sm, currState + 1);
                }
                break;

            case SMSTATE_STARTED + 1:
                //
                // Drive forward to block the opponent.
                //
                PIDDriveSetTarget(g_yPidDrive,
                                  0.0,
                                  distance,
                                  0.0,
                                  0,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE,
                                  10000);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

            default:
                SMStop(sm);
                PlayTone(440, 50);
                break;
        }
    }
}   //DoDefense
