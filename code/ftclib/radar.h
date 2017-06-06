#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="radar.h" />
///
/// <summary>
///     This module contains the library functions for the sonar sensor
///     on a rotating platform.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _RADAR_H
#define _RADAR_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                          MOD_RADAR

//
// Constants.
//
#define RADARF_SERVO_MOTOR              0x0001
#define RADARF_SMUX_SONAR               0x0002
#define RADARF_DIR_LOW                  0x0004
#define RADARF_SCAN_ENABLED             0x8000

#ifndef RADAR_SAMPLING_INTERVAL
    #define RADAR_SAMPLING_INTERVAL     1000
#endif
#ifndef MAX_SAMPLE_POINTS
    #define MAX_SAMPLE_POINTS           8
#endif

//
// Macros.
//

/**
 *  This macro checks if the radar is scanning.
 *
 *  @param r Points to the RADAR structure.
 *
 *  @return Returns true if the radar is scanning, false otherwise.
 */
#define RadarScanEnabled(r)             ((r).flags & RADARF_SCAN_ENABLED)

/**
 *  This macro checks if the radar motor is a servo.
 *
 *  @param r Points to the RADAR structure.
 *
 *  @return Returns true if the radar motor is a servo motor, false otherwise.
 */
#define RadarMotorIsServo(r)            ((r).flags & RADARF_SERVO_MOTOR)

/**
 *  This macro returns the sonar value. It will take care of whether the sonar
 *  sensor is directly connected to the NXT or via a sensor MUX.
 *
 *  @param r Points to the RADAR structure.
 *
 *  @return Returns the sonar sensor reading.
 */
#ifdef __HTSMUX_SUPPORT__
#define RadarReadSonar(r)               (((r).flags & RADARF_SMUX_SONAR)?      \
                                         USreadDist((tMUXSensor)((r).sonarID)):\
                                         USreadDist((tSensors)((r).sonarID)))
#else
#define RadarReadSonar(r)               USreadDist((tSensors)((r).sonarID))
#endif

/**
 *  This macro returns the next sampling angle.
 *
 *  @param r Points to the RADAR structure.
 *
 *  @return Returns the next sampling angle.
 */
#define RadarGetNextAngle(r)            ((r).sampleAngles[(r).nextSample])

/**
 *  This macro displays the Radar info: sample point, sample angle and
 *  sample distance.
 *
 *  @param p Points to the RADAR structure.
 *  @param n Specifies the LCD line number.
 */
#define RadarDisplayInfo(n,r)   nxtDisplayTextLine(n, "[%d] Angle=%5.1f",           \
                                                   (r).currSample,                  \
                                                   (r).sampleAngles[(r).currSample]);\
                                nxtDisplayTextLine(n + 1, "Dist=%6.1f",             \
                                                   (r).sampleData[(r).currSample]*  \
                                                   INCHES_PER_CM)

//
// Type definitions.
//
typedef struct
{
    int             sonarID;
    PIDMOTOR       *pidMotor;
#ifdef _SERVO_H
    SERVO          *servoMotor;
#endif
    unsigned long   samplingInterval;
    unsigned long   samplingTime;
    int             flags;
    int             numSamples;
    int             currSample;
    int             nextSample;
    float           targetAngle;
    float           sampleAngles[MAX_SAMPLE_POINTS];
    int             sampleData[MAX_SAMPLE_POINTS];
} RADAR;

/**
 *  This function is called by various RadarInit functions to do common
 *  initializations.
 *
 *  @param radar Points to the RADAR structure.
 *  @param sonarID Specifies the ID of the sonar sensor.
 *  @param samplingInterval Specifies the interval between samples.
 */
void
RadarCommonInit(
    RADAR &radar,
    int sonarID,
    unsigned long samplingInterval
    )
{
    TFuncName("RadarCommonInit");
    TLevel(INIT);
    TEnter();

    radar.sonarID = sonarID;
    radar.pidMotor = NULL;
#ifdef _SERVO_H
    radar.servoMotor = NULL;
#endif
    radar.samplingInterval = samplingInterval;
    radar.samplingTime = 0;
    radar.flags = 0;
    radar.numSamples = 0;
    radar.currSample = 0;
    radar.nextSample = 0;
    radar.targetAngle = -1.0;

    for (int i = 0; i < MAX_SAMPLE_POINTS; i++)
    {
        radar.sampleAngles[i] = 0.0;
        radar.sampleData[i] = 0;
    }

    TExit();
    return;
}   //RadarCommonInit

/**
 *  This function initializes the radar subsystem.
 *
 *  @param radar Points to the RADAR structure.
 *  @param sonarID Specifies the ID of the sonar sensor.
 *  @param pidMotor Points to the PIDMOTOR structure used for scanning.
 *  @param samplingInterval Optionally specifies the interval between
 *         samples.
 */
void
RadarInit(
    RADAR &radar,
    tSensors sonarID,
    PIDMOTOR *pidMotor,
    unsigned long samplingInterval = RADAR_SAMPLING_INTERVAL
    )
{
    TFuncName("RadarInit");
    TLevel(INIT);
    TEnter();

    RadarCommonInit(radar, (int)sonarID, samplingInterval);
    radar.pidMotor = pidMotor;

    TExit();
    return;
}   //RadarInit

#ifdef _SERVO_H
/**
 *  This function initializes the radar subsystem.
 *
 *  @param radar Points to the RADAR structure.
 *  @param sonarID Specifies the ID of the sonar sensor.
 *  @param servoMotor Points to the SERVO structure used for scanning.
 *  @param samplingInterval Optionally specifies the interval between
 *         samples.
 */
void
RadarInit(
    RADAR &radar,
    tSensors sonarID,
    SERVO *servoMotor,
    unsigned long samplingInterval = RADAR_SAMPLING_INTERVAL
    )
{
    TFuncName("RadarInit");
    TLevel(INIT);
    TEnter();

    RadarCommonInit(radar, (int)sonarID, samplingInterval);
    radar.servoMotor = servoMotor;
    radar.flags |= RADARF_SERVO_MOTOR;

    TExit();
    return;
}   //RadarInit
#endif

#ifdef __HTSMUX_SUPPORT__
/**
 *  This function initializes the radar subsystem.
 *
 *  @param radar Points to the RADAR structure.
 *  @param sonarID Specifies the ID of the sonar sensor.
 *  @param pidMotor Points to the PIDMOTOR structure used for scanning.
 *  @param samplingInterval Optionally specifies the interval between
 *         samples.
 */
void
RadarInit(
    RADAR &radar,
    tMUXSensor sonarID,
    PIDMOTOR *pidMotor,
    unsigned long samplingInterval = RADAR_SAMPLING_INTERVAL
    )
{
    TFuncName("RadarInit");
    TLevel(INIT);
    TEnter();

    RadarCommonInit(radar, (int)sonarID, samplingInterval);
    radar.pidMotor = pidMotor;
    radar.flags |= RADARF_SMUX_SONAR;

    TExit();
    return;
}   //RadarInit

#ifdef _SERVO_H
/**
 *  This function initializes the radar subsystem.
 *
 *  @param radar Points to the RADAR structure.
 *  @param sonarID Specifies the ID of the sonar sensor.
 *  @param servoMotor Points to the SERVO structure used for scanning.
 *  @param samplingInterval Optionally specifies the interval between
 *         samples.
 */
void
RadarInit(
    RADAR &radar,
    tMUXSensor sonarID,
    SERVO *servoMotor,
    unsigned long samplingInterval = RADAR_SAMPLING_INTERVAL
    )
{
    TFuncName("RadarInit");
    TLevel(INIT);
    TEnter();

    RadarCommonInit(radar, (int)sonarID, samplingInterval);
    radar.servoMotor = servoMotor;
    radar.flags |= RADARF_SERVO_MOTOR;
    radar.flags |= RADARF_SMUX_SONAR;

    TExit();
    return;
}   //RadarInit
#endif
#endif

/**
 *  This function adds a sampling angle.
 *
 *  @param radar Points to the RADAR structure.
 *  @param angle Specifies the sample angling in degrees.
 *
 *  @return Returns true if successful, false otherwise.
 */
bool
RadarAddSamplePoint(
    RADAR &radar,
    float angle
    )
{
    bool fSuccess = false;
    int i, j;

    TFuncName("RadarAddSamplePoint");
    TLevel(API);
    TEnter();

    if (radar.numSamples < MAX_SAMPLE_POINTS)
    {
        //
        // The sample angles must be sorted, so let's do an insertion
        // sort here.
        //
        for (i = 0; i < radar.numSamples; i++)
        {
            if (angle < radar.sampleAngles[i])
            {
                for (j = radar.numSamples - 1; j >= i; j--)
                {
                    radar.sampleAngles[j + 1] = radar.sampleAngles[j];
                }
                fSuccess = true;
                break;
            }
            else if (angle == radar.sampleAngles[i])
            {
                //
                // If sample angle already exists, don't add another one.
                //
                break;
            }
        }

        if (fSuccess || (i == radar.numSamples))
        {
            radar.sampleAngles[i] = angle;
            radar.numSamples++;
            fSuccess = true;
        }
    }

    TExitMsg(("=%d", fSuccess));
    return fSuccess;
}   //RadarAddSamplePoint

/**
 *  This function returns the distance data in inches read from the sonar
 *  sensor. If the radar is scanning, we will look up the angle from the
 *  sample angle table and return the corresponding sample data. If there
 *  is no angle found, it will return -1.0. If the radar is not scanning,
 *  we will rotate the radar to the specified angle and read the sonar data.
 *  Since it takes time to rotate the sonar to the correct angle and this
 *  function will not block and wait for the rotation to complete, it will
 *  return -1.0 as an indicator of no valid data returned. The caller should
 *  keep calling this function until the distance is not -1.0.
 *
 *  @param radar Points to the RADAR structure.
 *  @param angle Specifies the angle to sample the distance.
 *
 *  @return If success, it returns the sonar reading in inches.
 *          If Failure, it returns a negative value.
 */
float
RadarGetDistance(
    RADAR &radar,
    float angle
    )
{
    float distance = -1.0;

    TFuncName("RadarGetDistance");
    TLevel(API);
    TEnterMsg(("Angle=%5.1f", angle));

    if (RadarScanEnabled(radar))
    {
        for (int i = 0; i < radar.numSamples; i++)
        {
            if (angle == radar.sampleAngles[i])
            {
                distance = radar.sampleData[i]*INCHES_PER_CM;
                break;
            }
        }
    }
    else if (angle != radar.targetAngle)
    {
        //
        // We are not at the right angle, turn the radar to the specified
        // angle.
        //
        radar.targetAngle = angle;
        if (!RadarMotorIsServo(radar))
        {
            PIDMotorSetTarget(*radar.pidMotor, radar.targetAngle, false);
        }
#ifdef _SERVO_H
        else
        {
            ServoSetAngle(*radar.servoMotor, radar.targetAngle);
        }
#endif
        radar.samplingTime = GetMsecTime() + radar.samplingInterval;
    }
    else if (GetMsecTime() >= radar.samplingTime)
    {
        //
        // Read only if we have reached the sampling interval.
        //
        distance = RadarReadSonar(radar)*INCHES_PER_CM;
    }

    TExitMsg(("=%5.1f", distance));
    return distance;
}   //RadarGetDistance

/**
 *  This function enables or disables radar scanning.
 *
 *  @param radar Points to the RADAR structure.
 *  @param fEnable If true, enable the radar scanning, disable otherwise.
 */
void
RadarEnableScan(
    RADAR &radar,
    bool fEnable
    )
{
    TFuncName("RadarEnableScan");
    TLevel(API);
    TEnterMsg(("fEnable=%d", fEnable));

    if (fEnable)
    {
        radar.targetAngle = RadarGetNextAngle(radar);
        if (!RadarMotorIsServo(radar))
        {
            PIDMotorSetTarget(*radar.pidMotor, radar.targetAngle, false);
        }
#ifdef _SERVO_H
        else
        {
            ServoSetAngle(*radar.servoMotor, radar.targetAngle);
        }
#endif
        radar.samplingTime = GetMsecTime() + radar.samplingInterval;
        radar.flags |= RADARF_SCAN_ENABLED;
    }
    else
    {
        if (!RadarMotorIsServo(radar))
        {
            PIDMotorReset(*radar.pidMotor);
        }
        radar.flags &= ~RADARF_SCAN_ENABLED;
    }

    TExit();
    return;
}   //RadarEnableScan

/**
 *  This function performs the radar task where it will turn the sonar sensor
 *  left and right.
 *
 *  @param radar Points to the RADAR structure.
 *  @param lineNum Optionally specifies the LCD line number for debugging.
 */
void
RadarTask(
    RADAR &radar,
    int lineNum = -1
    )
{
    TFuncName("RadarTask");
    TLevel(TASK);
    TEnter();

    if (RadarScanEnabled(radar))
    {
        unsigned long currTime = GetMsecTime();

        if (currTime >= radar.samplingTime)
        {
            //
            // The servo has reached the next sample point, take a sample,
            // calculate the next sample point and program the servo to
            // go to the next sample point.
            //
            radar.currSample = radar.nextSample;
            radar.sampleData[radar.currSample] = RadarReadSonar(radar);
            if (lineNum != -1)
            {
                RadarDisplayInfo(lineNum, radar);
            }
            //
            // Calculate the next sample point.
            //
            if (radar.flags & RADARF_DIR_LOW)
            {
                if (radar.nextSample == 0)
                {
                    //
                    // Reached the low limit, reverse.
                    //
                    radar.nextSample = 1;
                    radar.flags &= ~RADARF_DIR_LOW;
                }
                else
                {
                    radar.nextSample--;
                }
            }
            else
            {
                if (radar.nextSample == radar.numSamples - 1)
                {
                    //
                    // Reached the high limit, reverse.
                    //
                    radar.nextSample = radar.numSamples - 2;
                    radar.flags |= RADARF_DIR_LOW;
                }
                else
                {
                    radar.nextSample++;
                }
            }

            radar.targetAngle = RadarGetNextAngle(radar);
            if (!RadarMotorIsServo(radar))
            {
                PIDMotorSetTarget(*radar.pidMotor, radar.targetAngle, false);
            }
#ifdef _SERVO_H
            else
            {
                ServoSetAngle(*radar.servoMotor, radar.targetAngle);
            }
#endif
            radar.samplingTime = currTime + radar.samplingInterval;
        }
    }

    TExit();
    return;
}   //RadarTask

#endif  //ifndef _RADAR_H
