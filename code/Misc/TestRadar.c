#pragma config(Hubs,  S1, HTServo,  none,     none,     none)
#pragma config(Sensor, S2,     radarSonar,     sensorI2CCustom)
#pragma config(Motor,  motorA,          radarMotor,    tmotorNXT, openLoop, encoder)
#pragma config(Servo,  srvo_S1_C1_1,    radarServo,           tServoStandard)
#pragma config(Servo,  srvo_S1_C1_2,    servo2,               tServoNone)
#pragma config(Servo,  srvo_S1_C1_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C1_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C1_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C1_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "..\RobotCDrivers\drivers\lego-ultrasound.h"

#define _USE_SERVO
#define _NO_SCANNING

#include "..\ftclib\trcdefs.h"
#include "..\ftclib\dbgtrace.h"
#include "..\ftclib\sm.h"
#ifdef _USE_SERVO
  #include "..\ftclib\servo.h"
#endif
#include "..\ftclib\touch.h"
#include "..\ftclib\pidctrl.h"
#include "..\ftclib\pidmotor.h"
#include "..\ftclib\radar.h"

#ifdef _USE_SERVO
SERVO       g_radarServo;
#else
PIDCTRL     g_radarPidCtrl;
PIDMOTOR    g_radarMotor;
#endif
RADAR       g_radar;

void TouchEvent(TOUCH &touch, bool fActive)
{
}   //TouchEvent

float PIDCtrlGetInput(PIDCTRL &pidCtrl)
{
#ifdef _USE_SERVO
    return 0.0;
#else
    return nMotorEncoder[radarMotor];
#endif
}   //PIDCtrlGetInput

task main()
{
#ifdef _NO_SCANNING
    float dist = 0.0;
    float angle = 0.0;
    float angleInc = 10.0;
#endif

#ifdef _USE_SERVO
    ServoInit(g_radarServo, radarServo, 0, 180.0, 90.0);
    RadarInit(g_radar, radarSonar, &g_radarServo, 500);
#else
    nMotorEncoder[radarMotor] = 0;
    PIDCtrlInit(g_radarPidCtrl,
                1.0, 0.0, 0.0, 3.0, 200,
                (PIDCTRLO_ABS_SETPT |
                 PIDCTRLO_NO_OSCILLATE));
    PIDMotorInit(g_radarMotor, radarMotor, g_radarPidCtrl);
    RadarInit(g_radar, radarSonar,&g_radarMotor, 500);
#endif
    RadarAddSamplePoint(g_radar, -90.0);
    RadarAddSamplePoint(g_radar, -45.0);
    RadarAddSamplePoint(g_radar, 0.0);
    RadarAddSamplePoint(g_radar, 45.0);
    RadarAddSamplePoint(g_radar, 90.0);
#ifdef _NO_SCANNING
    RadarEnableScan(g_radar, false);
#else
    RadarEnableScan(g_radar, true);
#endif
    while (true)
    {
        nxtDisplayTextLine(0, "Enc=%d", nMotorEncoder[radarMotor]);
#ifdef _NO_SCANNING
        dist = RadarGetDistance(g_radar, angle);
        if (dist != -1.0)
        {
            nxtDisplayTextLine(1, "A=%5.1f,D=%d", angle, dist);
            if ((angle <= -90.0) || (angle >= 90.0))
            {
                angleInc *= -1.0;
                angle += 2*angleInc;
            }
            else
            {
                angle += angleInc;
            }
        }
#endif
#ifdef _USE_SERVO
        ServoTask(g_radarServo);
#else
        PIDMotorTask(g_radarMotor);
#endif
        RadarTask(g_radar, 1);
        wait1Msec(20);
    }
}   //main
