#pragma config(Hubs,  S1, HTServo,  none,     none,     none)
#pragma config(Sensor, S1,     colorSensor,    sensorI2CMuxController)
#pragma config(Servo,  srvo_S1_C1_1,    rollerServo,          tServoContinuousRotation)
#pragma config(Servo,  srvo_S1_C1_2,    servo2,               tServoNone)
#pragma config(Servo,  srvo_S1_C1_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C1_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C1_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C1_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

task main()
{
    while (true)
    {
        if (nNxtButtonPressed == 1)
        {
            servo[rollerServo] = 0;
        }
        else if (nNxtButtonPressed == 2)
        {
            servo[rollerServo] = 255;
        }
        else
        {
            servo[rollerServo] = 128;
        }
        wait1Msec(100);
    }
}
