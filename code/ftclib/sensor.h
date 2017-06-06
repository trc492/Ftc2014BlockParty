#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="sensor.h" />
///
/// <summary>
///     This module contains the library functions to handle various sensors.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _SENSOR_H
#define _SENSOR_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_SENSOR

//
// Constants.
//
#define SENSORZONE_LOW          0
#define SENSORZONE_MID          1
#define SENSORZONE_HIGH         2
#define NUM_SENSOR_ZONES        3

#define SENSORO_INVERSE         0x0001
#define SENSORF_ENABLED         0x0001

//
// Macros.
//

/**
 * This macro returns the current zone detected.
 *
 *  @param s Points to the SENSOR structure.
 *
 *  @return Returns the current sensor zone.
 */
#define SensorGetZone(s)        ((s).zone)

//
// Type definitions.
//
typedef struct
{
    float       lowThreshold;
    float       highThreshold;
    float       value;
    int         options;
    int         flags;
    int         zone;
} SENSOR;

//
// Callback function prototypes.
//

/**
 *  This callback function is called when the sensor task needs to read the
 *  sensor value.
 *
 *  @param sensor Points to the SENSOR structure.
 *
 *  @return Returns the current sensor value.
 */
float
SensorGetValue(
    SENSOR &sensor
    );

/**
 *  This callback function is called when the sensor value crosses to a
 *  different zone.
 *
 *  @param sensor Points to the SENSOR structure.
 *  @param zone Specifying the current sensor zone.
 */
void
SensorEvent(
    SENSOR &sensor,
    int zone
    );

/**
 *  This function updates the sensor reading and determines the corresponding
 *  zone.
 *
 *  @param sensor Points to the SENSOR structure.
 */
void
SensorUpdate(
    SENSOR &sensor
    )
{
    TFuncName("SensorUpdate");
    TLevel(FUNC);
    TEnter();

    sensor.value = SensorGetValue(sensor);
    if (sensor.value <= sensor.lowThreshold)
    {
        sensor.zone = (sensor.options & SENSORO_INVERSE)?
                        SENSORZONE_HIGH: SENSORZONE_LOW;
    }
    else if (sensor.value <= sensor.highThreshold)
    {
        sensor.zone = SENSORZONE_MID;
    }
    else
    {
        sensor.zone = (sensor.options & SENSORO_INVERSE)?
                        SENSORZONE_LOW: SENSORZONE_HIGH;
    }

    TExit();
    return;
}   //SensorUpdate

/**
 *  This function initializes the sensor system.
 *
 *  @param sensor Points to the SENSOR structure.
 *  @param lowThreshold Specifies the low threshold value.
 *  @param highThreshold Specifies the high threshold value.
 *  @param options Optionally specifies the sensor options:
 *         SENSORO_INVERSE - Specifies sensor reading is inverse.
 */
void
SensorInit(
    SENSOR &sensor,
    float lowThreshold,
    float highThreshold,
    int   options = 0
    )
{
    TFuncName("SensorInit");
    TLevel(INIT);
    TEnter();

    sensor.lowThreshold = lowThreshold;
    sensor.highThreshold = highThreshold;
    sensor.options = options;
    sensor.flags = 0;
    SensorUpdate(sensor);

    TExit();
    return;
}   //SensorInit

/**
 *  This function enables or disables the monitoring of the sensor.
 *
 *  @param sensor Points to the SENSOR structure.
 *  @param fEnabled Specifies whether to enable or disable sensor monitoring.
 */
void
SensorSetEnabled(
    SENSOR &sensor,
    bool fEnabled
    )
{
    TFuncName("SensorSetEnabled");
    TLevel(API);
    TEnterMsg(("fEnabled=%d", (byte)fEnabled));

    if (fEnabled)
    {
        sensor.flags |= SENSORF_ENABLED;
    }
    else
    {
        sensor.flags &= ~SENSORF_ENABLED;
    }

    TExit();
    return;
}   //SensorSetEnabled

/**
 *  This function processes the sensor reading and sends a trigger event if
 *  necessary.
 *
 *  @param sensor Points to the SENSOR structure.
 */
void
SensorTask(
    SENSOR &sensor
    )
{
    TFuncName("SensorTask");
    TLevel(TASK);
    TEnter();

    if (sensor.flags & SENSORF_ENABLED)
    {
        int prevZone = sensor.zone;
        SensorUpdate(sensor);
        if (sensor.zone != prevZone)
        {
            //
            // We have crossed to another zone, let's send a sensor event.
            //
            SensorEvent(sensor, sensor.zone);
        }
    }

    TExit();
    return;
}   //SensorTask

#endif  //ifndef _SENSOR_H
