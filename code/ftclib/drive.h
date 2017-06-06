#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="drive.h" />
///
/// <summary>
///     This module contains the library functions for the drive subsystem.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _DRIVE_H
#define _DRIVE_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_DRIVE

//
// Constants.
//
#define DRIVEF_STATE_MASK       0x00ff
#define DRIVEF_ON               0x0001
#define DRIVEF_STALLED          0x0002
#define DRIVEF_STALL_PROTECT_ON 0x0100
#define DRIVEF_FOUR_MOTORS      0x0200
#define DRIVEF_SWERVE           0x0400

#define DRIVE_MIN_STALL_POWER   20
#define DRIVE_STALL_TIME        2000    //2 seconds

#define MAX_NUM_MOTORS          4
#define IDX_FRONT_LEFT          0
#define IDX_FRONT_RIGHT         1
#define IDX_REAR_LEFT           2
#define IDX_REAR_RIGHT          3

#define CRABDIR_NONE            0
#define CRABDIR_LEFT            1
#define CRABDIR_RIGHT           2

//
// Macros.
//

/**
 *  This macro checks if the drive subsystem is stalled.
 *
 *  @param d Points to the DRIVE structure.
 *
 *  @return Returns true if detected stall condition.
 */
#define DriveIsStalled(d)       ((d).flags & DRIVEF_STALLED)

/**
 *  This macro checks if the drive subsystem is using swerve drive.
 *
 *  @param d Points to the DRIVE structure.
 *
 *  @return Returns true if it is swerve drive.
 */
#define DriveIsSwerve(d)        ((d).flags & DRIVEF_SWERVE)

/**
 *  This macro checks if the drive subsystem is 4-motor drive.
 *
 *  @param d Points to the DRIVE structure.
 *
 *  @return Returns true if it is 4-motor drive.
 */
#define DriveHas4Motors(d)      ((d).flags & DRIVEF_FOUR_MOTORS)

/**
 *  This macro returns the robot's X position using the motor encoders.
 *
 *  @param d Points to the DRIVE structure.
 *
 *  @return Returns the X position in encoder clicks.
 */
#define DriveGetXPos(d)         ((d).xPos)

/**
 *  This macro returns the robot's Y position using the motor encoders.
 *
 *  @param d Points to the DRIVE structure.
 *
 *  @return Returns the Y position in encoder clicks.
 */
#define DriveGetYPos(d)         ((d).yPos)

/**
 *  This macro returns the robot's Rotation position using the motor encoders.
 *
 *  @param d Points to the DRIVE structure.
 *
 *  @return Returns the Rotation position in encoder clicks.
 */
#define DriveGetRotPos(d)       ((d).rotPos)

//
// Type definitions.
//
typedef struct
{
    int             flags;
    tMotor          motorIDs[MAX_NUM_MOTORS];
    int             motorPowers[MAX_NUM_MOTORS];
    long            motorEncoders[MAX_NUM_MOTORS];
    float           motorSpeeds[MAX_NUM_MOTORS];
    unsigned long   prevTime;
    unsigned long   stallTimer;
    float           xPos;
    float           yPos;
    float           rotPos;
#ifdef _SERVO_H
    SERVO          *servos[MAX_NUM_MOTORS];
    float           lowAngleLimit;
    float           highAngleLimit;
#endif
} DRIVE;

/**
 *  This function resets the drive system.
 *
 *  @param drive Points to the DRIVE structure to be reset.
 */
void
DriveReset(
    DRIVE &drive
    )
{
    TFuncName("DriveReset");
    TLevel(API);
    TEnter();

    int numMotors = DriveHas4Motors(drive)? 4: 2;

    //
    // Stop the motors.
    //
    for (int i = 0; i < numMotors; i++)
    {
        drive.motorPowers[i] = 0;
        drive.motorEncoders[i] = 0;
        drive.motorSpeeds[i] = 0.0;
        motor[drive.motorIDs[i]] = 0;
        nMotorEncoder[drive.motorIDs[i]] = 0;
    }
    drive.prevTime = GetMsecTime();
    drive.stallTimer = 0;
    drive.flags &= ~DRIVEF_STATE_MASK;

    TExit();
    return;
}   //DriveReset

/**
 *  This function initializes the drive system for 2-motor drive.
 *
 *  @param drive Points to the DRIVE structure.
 *  @param leftMotor Specifies the left motor.
 *  @param rightMotor Specifies the right motor.
 */
void
DriveInit(
    DRIVE &drive,
    tMotor leftMotor,
    tMotor rightMotor
    )
{
    TFuncName("DriveInit");
    TLevel(INIT);
    TEnter();

    for (int i = 0; i < MAX_NUM_MOTORS; i++)
    {
        drive.motorIDs[i] = (tMotor)-1;
#ifdef _SERVO_H
        drive.servos[i] = NULL;
#endif
        drive.motorPowers[i] = 0;
        drive.motorEncoders[i] = 0;
        drive.motorSpeeds[i] = 0.0;
    }
    drive.motorIDs[IDX_FRONT_LEFT] = leftMotor;
    drive.motorIDs[IDX_FRONT_RIGHT] = rightMotor;
    drive.prevTime = GetMsecTime();
    drive.stallTimer = 0;
    drive.flags = 0;
    drive.xPos = 0.0;
    drive.yPos = 0.0;
    drive.rotPos = 0.0;

    TExit();
    return;
}   //DriveInit

/**
 *  This function initializes the drive system for 4-motor drive.
 *
 *  @param drive Points to the DRIVE structure.
 *  @param frontLeftMotor Specifies the front left motor.
 *  @param frontRightMotor Specifies the front right motor.
 *  @param rearLeftMotor Specifies the rear left motor.
 *  @param rearRightMotor Specifies the rear right motor.
 */
void
DriveInit(
    DRIVE &drive,
    tMotor frontLeftMotor,
    tMotor frontRightMotor,
    tMotor rearLeftMotor,
    tMotor rearRightMotor
    )
{
    TFuncName("DriveInit");
    TLevel(INIT);
    TEnter();

    DriveInit(drive, frontLeftMotor, frontRightMotor);
    drive.motorIDs[IDX_REAR_LEFT] = rearLeftMotor;
    drive.motorIDs[IDX_REAR_RIGHT] = rearRightMotor;
    drive.flags |= DRIVEF_FOUR_MOTORS;

    TExit();
    return;
}   //DriveInit

#ifdef _SERVO_H
/**
 *  This function initializes the drive system for 4-motor and 4-servo drive.
 *
 *  @param drive Points to the DRIVE structure.
 *  @param frontLeftMotor Specifies the front left motor.
 *  @param frontRightMotor Specifies the front right motor.
 *  @param rearLeftMotor Specifies the rear left motor.
 *  @param rearRightMotor Specifies the rear right motor.
 *  @param frontLeftServo Specifies the front left servo motor.
 *  @param frontRightServo Specifies the front right servo motor.
 *  @param rearLeftServo Specifies the rear left servo motor.
 *  @param rearRightServo Specifies the rear right servo motor.
 *  @param lowAngleLimit Optionally specifies the servo low angle limit.
 *  @param highAngleLimit Optionally specifies the servo high angle limit.
 */
void
DriveInit(
    DRIVE  &drive,
    tMotor  frontLeftMotor,
    tMotor  frontRightMotor,
    tMotor  rearLeftMotor,
    tMotor  rearRightMotor,
    SERVO  &frontLeftServo,
    SERVO  &frontRightServo,
    SERVO  &rearLeftServo,
    SERVO  &rearRightServo,
    float   lowAngleLimit = -90.0,
    float   highAngleLimit = 90.0
    )
{
    TFuncName("DriveInit");
    TLevel(INIT);
    TEnter();

    DriveInit(drive, frontLeftMotor, frontRightMotor,
              rearLeftMotor, rearRightMotor);
    drive.servos[IDX_FRONT_LEFT] = &frontLeftServo;
    drive.servos[IDX_FRONT_RIGHT] = &frontRightServo;
    drive.servos[IDX_REAR_LEFT] = &rearLeftServo;
    drive.servos[IDX_REAR_RIGHT] = &rearRightServo;
    drive.lowAngleLimit = lowAngleLimit;
    drive.highAngleLimit = highAngleLimit;
    drive.flags |= DRIVEF_SWERVE;

    TExit();
    return;
}   //DriveInit
#endif

/**
 *  This function enables or disables stall protection.
 *
 *  @param drive Points to the DRIVE structure.
 *  @param fOn If true, enables stall protection.
 */
void
DriveStallProtect(
    DRIVE &drive,
    bool fOn
    )
{
    TFuncName("DriveStallProtect");
    TLevel(API);
    TEnterMsg(("fOn=%d", (byte)fOn));

    if (fOn)
    {
        drive.flags |= DRIVEF_STALL_PROTECT_ON;
    }
    else
    {
        drive.flags &= ~DRIVEF_STALL_PROTECT_ON;
    }

    TExit();
    return;
}   //DriveStallProtect

/**
 *  This function sets power of the motors for tank drive.
 *
 *  @param drive Points to the DRIVE structure.
 *  @param leftPower Specifies the left motor power.
 *  @param rightPower Specifies the right motor power.
 */
void
DriveTank(
    DRIVE &drive,
    int leftPower,
    int rightPower
    )
{
    TFuncName("DriveTank");
    TLevel(API);
    TEnterMsg(("Left=%d,Right=%d", leftPower, rightPower));

    drive.motorPowers[IDX_FRONT_LEFT] =
        BOUND(leftPower, MOTOR_MIN_VALUE, MOTOR_MAX_VALUE);
    drive.motorPowers[IDX_FRONT_RIGHT] =
        BOUND(rightPower, MOTOR_MIN_VALUE, MOTOR_MAX_VALUE);
    if (DriveHas4Motors(drive))
    {
        drive.motorPowers[IDX_REAR_LEFT] =
            drive.motorPowers[IDX_FRONT_LEFT];
        drive.motorPowers[IDX_REAR_RIGHT] =
            drive.motorPowers[IDX_FRONT_RIGHT];
    }
    drive.flags |= DRIVEF_ON;

    TExit();
    return;
}   //DriveTank

/**
 *  This function sets power of the motors for arcade drive.
 *
 *  @param drive Points to the DRIVE structure.
 *  @param drivePower Specifies the drive power.
 *  @param turnPower Specifies the turn power.
 *  @param crabDir Optionally specifies the crabbing direction:
 *          CRABDIR_NONE: No crabbing (default).
 *          CRABDIR_LEFT: Crab to left.
 *          CRABDIR_RIGHT: Crab to right.
 */
void
DriveArcade(
    DRIVE &drive,
    int drivePower,
    int turnPower,
    int crabDir = CRABDIR_NONE
    )
{
    TFuncName("DriveArcade");
    TLevel(API);
    TEnterMsg(("Drive=%d,Turn=%d,Crab=%d", drivePower, turnPower, crabDir));

    int leftPower, rightPower;

    drivePower = BOUND(drivePower, MOTOR_MIN_VALUE, MOTOR_MAX_VALUE);
    turnPower = BOUND(turnPower, MOTOR_MIN_VALUE, MOTOR_MAX_VALUE);
    if (drivePower + turnPower > MOTOR_MAX_VALUE)
    {
        //
        // Forward right:
        //  left = drive + turn - (drive + turn - MOTOR_MAX_VALUE)
        //  right = drive - turn - (drive + turn - MOTOR_MAX_VALUE)
        //
        leftPower = MOTOR_MAX_VALUE;
        rightPower = -2*turnPower + MOTOR_MAX_VALUE;
    }
    else if (drivePower - turnPower > MOTOR_MAX_VALUE)
    {
        //
        // Forward left:
        //  left = drive + turn - (drive - turn - MOTOR_MAX_VALUE)
        //  right = drive - turn - (drive - turn - MOTOR_MAX_VALUE)
        //
        leftPower = 2*turnPower + MOTOR_MAX_VALUE;
        rightPower = MOTOR_MAX_VALUE;
    }
    else if (drivePower + turnPower < MOTOR_MIN_VALUE)
    {
        //
        // Backward left:
        //  left = drive + turn - (drive + turn - MOTOR_MIN_VALUE)
        //  right = drive - turn - (drive + turn - MOTOR_MIN_VALUE)
        //
        leftPower = MOTOR_MIN_VALUE;
        rightPower = -2*turnPower + MOTOR_MIN_VALUE;
    }
    else if (drivePower - turnPower < MOTOR_MIN_VALUE)
    {
        //
        // Backward right:
        //  left = drive + turn - (drive - turn - MOTOR_MIN_VALUE)
        //  right = drive - turn - (drive - turn - MOTOR_MIN_VALUE)
        //
        leftPower = 2*turnPower + MOTOR_MIN_VALUE;
        rightPower = MOTOR_MIN_VALUE;
    }
    else
    {
        leftPower = drivePower + turnPower;
        rightPower = drivePower - turnPower;
    }

    if (DriveHas4Motors(drive))
    {
        if (crabDir == CRABDIR_LEFT)
        {
            drive.motorPowers[IDX_FRONT_LEFT] = rightPower;
            drive.motorPowers[IDX_FRONT_RIGHT] = rightPower;
            drive.motorPowers[IDX_REAR_LEFT] = leftPower;
            drive.motorPowers[IDX_REAR_RIGHT] = leftPower;
        }
        else if (crabDir == CRABDIR_RIGHT)
        {
            drive.motorPowers[IDX_FRONT_LEFT] = leftPower;
            drive.motorPowers[IDX_FRONT_RIGHT] = leftPower;
            drive.motorPowers[IDX_REAR_LEFT] = rightPower;
            drive.motorPowers[IDX_REAR_RIGHT] = rightPower;
        }
        else
        {
            drive.motorPowers[IDX_FRONT_LEFT] = leftPower;
            drive.motorPowers[IDX_FRONT_RIGHT] = rightPower;
            drive.motorPowers[IDX_REAR_LEFT] = leftPower;
            drive.motorPowers[IDX_REAR_RIGHT] = rightPower;
        }
    }
    else
    {
        drive.motorPowers[IDX_FRONT_LEFT] = leftPower;
        drive.motorPowers[IDX_FRONT_RIGHT] = rightPower;
    }

    drive.flags |= DRIVEF_ON;

    TExit();
    return;
}   //DriveArcade

#ifdef _SERVO_H
/**
 *  This function sets the wheel angles for swerve drive.
 *
 *  @param drive Points to the DRIVE structure.
 *  @param frontLeftAngle Specifies the angle of the front left wheel.
 *  @param frontRightAngle Specifies the angle of the front right wheel.
 *  @param rearLeftAngle Specifies the angle of the rear left wheel.
 *  @param rearRightAngle Specifies the angle of the rear right wheel.
 */
void
DriveSwerveSetAngles(
    DRIVE &drive,
    float frontLeftAngle,
    float frontRightAngle,
    float rearLeftAngle,
    float rearRightAngle
    )
{
    TFuncName("DriveSwerveSetAngles");
    TLevel(API);
    TEnter();

    if (DriveIsSwerve(drive))
    {
        frontLeftAngle = BOUND(frontLeftAngle,
                               drive.lowAngleLimit, drive.highAngleLimit);
        frontRightAngle = BOUND(frontRightAngle,
                                drive.lowAngleLimit, drive.highAngleLimit);
        rearLeftAngle = BOUND(rearLeftAngle,
                              drive.lowAngleLimit, drive.highAngleLimit);
        rearRightAngle = BOUND(rearRightAngle,
                               drive.lowAngleLimit, drive.highAngleLimit);
        ServoSetAngle(*drive.servos[IDX_FRONT_LEFT], frontLeftAngle);
        ServoSetAngle(*drive.servos[IDX_FRONT_RIGHT], frontRightAngle);
        ServoSetAngle(*drive.servos[IDX_REAR_LEFT], rearLeftAngle);
        ServoSetAngle(*drive.servos[IDX_REAR_RIGHT], rearRightAngle);
    }

    TExit();
    return;
}   //DriveSwerveSetAngles

/**
 *  This function sets power of the motors for swerve drive in cartesian
 *  coordinate system.
 *
 *  @param drive Points to the DRIVE structure.
 *  @param x Specifies the x speed.
 *  @param y Specifies the y speed.
 *  @param rot Specifies the rotaton speed.
 *  @param diffRot Optionally specifies if using left/right differential
 *                 to rotate.
 *  @param retNeutral Optionally specifies if the wheels will return to
 *         neutral angle when we stop driving.
 *  @param noDrive Optionally specifies if the drive motors will not move when
 *         true. This is useful for turning the servos only so the wheels are
 *         in the correct direction before we start driving.
 */
void
DriveSwerve(
    DRIVE &drive,
    int x,
    int y,
    int rot,
    bool diffRot = false,
    bool retNeutral = false,
    bool noDrive = false
    )
{
    TFuncName("DriveSwerve");
    TLevel(API);
    TEnterMsg(("x=%d,y=%d,rot=%d", x, y, rot));

    x = BOUND(x, MOTOR_MIN_VALUE, MOTOR_MAX_VALUE);
    y = BOUND(y, MOTOR_MIN_VALUE, MOTOR_MAX_VALUE);
    rot = BOUND(rot, MOTOR_MIN_VALUE, MOTOR_MAX_VALUE);

    //
    // Calculate the magnitude and scale to a maximum of MAX_MOTOR_VALUE.
    //
    int mag = BOUND((int)sqrt(x*x + y*y), MOTOR_MIN_VALUE, MOTOR_MAX_VALUE);
    float angle;

    if (mag == 0)
    {
        angle = 0.0;
    }
    else
    {
        angle = 90.0 - atan2(y, x)*180.0/PI;
        if (angle > 90.0)
        {
            angle -= 180.0;
            mag = -mag;
        }
    }

    if ((mag == 0) && (rot != 0) && !diffRot)
    {
        //
        // We are doing rotate only. We will set the servo angles to
        // a fixed 45-degree diamond formation for in-place rotation.
        // We will set the driving wheel power proportional to the
        // rotation magnitude. We must apply some power to the wheel
        // motors because, first of all, the user is expecting the
        // robot to turn. Without wheel power, the robot won't turn.
        // Secondly, with the robot not moving, the friction on the
        // wheels may be too big to even turn the servos.
        //
        ServoSetAngle(*drive.servos[IDX_FRONT_LEFT],
                      NORMALIZE(45.0,
                                -90.0, 90.0,
                                drive.lowAngleLimit,
                                drive.highAngleLimit));
        ServoSetAngle(*drive.servos[IDX_FRONT_RIGHT],
                      NORMALIZE(-45.0,
                                -90.0, 90.0,
                                drive.lowAngleLimit,
                                drive.highAngleLimit));
        ServoSetAngle(*drive.servos[IDX_REAR_LEFT],
                      NORMALIZE(-45.0,
                                -90.0, 90.0,
                                drive.lowAngleLimit,
                                drive.highAngleLimit));
        ServoSetAngle(*drive.servos[IDX_REAR_RIGHT],
                      NORMALIZE(45.0,
                                -90.0, 90.0,
                                drive.lowAngleLimit,
                                drive.highAngleLimit));

        drive.motorPowers[IDX_FRONT_LEFT] =
        drive.motorPowers[IDX_REAR_LEFT] =
            noDrive? 0: rot;
        drive.motorPowers[IDX_FRONT_RIGHT] =
        drive.motorPowers[IDX_REAR_RIGHT] =
            noDrive? 0: -rot;
    }
    else if (diffRot)
    {
        //
        // Do rotation by setting different speed of the left and right
        // wheels.
        //
        float prevAngle = ServoGetAngle(*drive.servos[IDX_FRONT_LEFT]);
        if ((x != 0) && (y == 0) && (rot == 0) &&
            (abs(angle - prevAngle) == 180.0))
        {
            //
            // Crab only mode. In this scenario, make sure we don't
            // turn the wheels 180-degree when changing direction.
            // We can just spin the wheels the other way. This will
            // prevent the robot from twitching.
            //
            angle = prevAngle;
            mag = -mag;
        }

        if ((angle != prevAngle) && ((mag != 0.0) || retNeutral))
        {
            ServoSetAngle(*drive.servos[IDX_FRONT_LEFT], angle);
            ServoSetAngle(*drive.servos[IDX_FRONT_RIGHT], angle);
            ServoSetAngle(*drive.servos[IDX_REAR_LEFT], angle);
            ServoSetAngle(*drive.servos[IDX_REAR_RIGHT], angle);
        }
        DriveArcade(drive, mag, rot,
                    (angle == -90.0)? CRABDIR_LEFT:
                    (angle == 90.0)? CRABDIR_RIGHT: CRABDIR_NONE);
    }
    else
    {
        rot = NORMALIZE(rot,
                        MOTOR_MIN_VALUE, MOTOR_MAX_VALUE,
                        -45.0, 45.0);

        float frontAngle = NORMALIZE(BOUND(angle + rot, -90.0, 90.0),
                                     -90.0,
                                     90.0,
                                     drive.lowAngleLimit,
                                     drive.highAngleLimit);
        float rearAngle = NORMALIZE(BOUND(angle - rot, -90.0, 90.0),
                                    -90.0,
                                    90.0,
                                    drive.lowAngleLimit,
                                    drive.highAngleLimit);

        if ((x != 0) && (y == 0) && (rot == 0) &&
            (abs(frontAngle -
                 ServoGetAngle(*drive.servos[IDX_FRONT_LEFT])) == 180.0))
        {
            //
            // Crab only mode. In this scenario, make sure we don't
            // turn the wheels 180-degree when changing direction.
            // We can just spin the wheels the other way. This will
            // prevent the robot from twitching.
            //
            frontAngle = -frontAngle;
            rearAngle = -rearAngle;
            mag = -mag;
        }

        if (mag != 0 || retNeutral)
        {
            //
            // Turn the wheels only if we are moving. This will leave the
            // wheels at the previous angles and not turn the wheels to
            // forward direction if we let go of the joystick, for example.
            // This will eliminate unnecessary twitching of the robot.
            //
            ServoSetAngle(*drive.servos[IDX_FRONT_LEFT], frontAngle);
            ServoSetAngle(*drive.servos[IDX_FRONT_RIGHT], frontAngle);
            ServoSetAngle(*drive.servos[IDX_REAR_LEFT], rearAngle);
            ServoSetAngle(*drive.servos[IDX_REAR_RIGHT], rearAngle);
        }

        for (int i = 0; i < MAX_NUM_MOTORS; i++)
        {
            drive.motorPowers[i] = noDrive? 0: mag;
        }
    }

    drive.flags |= DRIVEF_ON;

    TExit();
    return;
}   //DriveSwerve
#endif

/**
 *  This function sets power of the motors for mecanum drive in cartesian
 *  coordinate system.
 *
 *  @param drive Points to the DRIVE structure.
 *  @param x Specifies the x speed.
 *  @param y Specifies the y speed.
 *  @param rot Specifies the rotaton speed.
 */
void
DriveMecanumCartesian(
    DRIVE &drive,
    int x,
    int y,
    int rot
    )
{
    TFuncName("MecanumCartesian");
    TLevel(API);
    TEnterMsg(("x=%d,y=%d,rot=%d", x, y, rot));

#ifdef _SERVO_H
    if (drive.flags & DRIVEF_SWERVE)
    {
        DriveSwerve(drive, x, y, rot);
    }
    else
#endif
    if (DriveHas4Motors(drive))
    {
        int mag, maxMag, i;

        drive.motorPowers[IDX_FRONT_LEFT] = x + y + rot;
        drive.motorPowers[IDX_FRONT_RIGHT] = -x + y - rot;
        drive.motorPowers[IDX_REAR_LEFT] = -x + y + rot;
        drive.motorPowers[IDX_REAR_RIGHT] = x + y - rot;
        //
        // Normalize
        //
        maxMag = abs(drive.motorPowers[0]);
        for (i = 1; i < MAX_NUM_MOTORS; i++)
        {
            mag = abs(drive.motorPowers[i]);
            if (mag > maxMag)
            {
                maxMag = mag;
            }
        }

        if (maxMag > MOTOR_MAX_VALUE)
        {
            for (i = 0; i < MAX_NUM_MOTORS; i++)
            {
                drive.motorPowers[i] =
                    (drive.motorPowers[i]*MOTOR_MAX_VALUE)/maxMag;
            }
        }
    }
    else
    {
        //
        // Mecanum drive is only possible with 4 motors. For 2 motors, we
        // do arcade drive instead.
        //
        DriveArcade(drive, y, rot);
    }

    drive.flags |= DRIVEF_ON;

    TExit();
    return;
}   //DriveMecanumCartesian

/**
 *  This function sets power of the motors for mecanum drive in polar
 *  coordinate system.
 *
 *  @param drive Points to the DRIVE structure.
 *  @param mag Specifies the magnitude.
 *  @param dir Specifies the direction.
 *  @param rot Specifies the rotaton.
 */
void
DriveMecanumPolar(
    DRIVE &drive,
    int mag,
    int dir,
    int rot
    )
{
    TFuncName("MecanumPolar");
    TLevel(API);
    TEnterMsg(("m=%d,d=%d,r=%d", mag, dir, rot));

    if (DriveHas4Motors(drive))
    {
        float magnitude = (BOUND(mag, MOTOR_MIN_VALUE, MOTOR_MAX_VALUE)*
                           sqrt(2.0))/MOTOR_MAX_VALUE;
        float rotation = (float)BOUND(rot, MOTOR_MIN_VALUE, MOTOR_MAX_VALUE)/
                         MOTOR_MAX_VALUE;
        float dirInRad = (dir + 45.0)*PI/180.0;
        float sinD = sin(dirInRad);
        float cosD = cos(dirInRad);
        float powers[MAX_NUM_MOTORS];

        powers[IDX_FRONT_LEFT] = sinD*magnitude + rotation;
        powers[IDX_FRONT_RIGHT] = cosD*magnitude - rotation;
        powers[IDX_REAR_LEFT] = cosD*magnitude + rotation;
        powers[IDX_REAR_RIGHT] = sinD*magnitude - rotation;
        //
        // Normalize
        //
        float maxMag = abs(powers[0]);
        float mag;
        int i;

        for (i = 1; i < MAX_NUM_MOTORS; i++)
        {
            mag = abs(powers[i]);
            if (mag > maxMag)
            {
                maxMag = mag;
            }
        }

        for (i = 0; i < MAX_NUM_MOTORS; i++)
        {
            if (maxMag > 1.0)
            {
                drive.motorPowers[i] = (int)
                    (powers[i]*MOTOR_MAX_VALUE/maxMag);
            }
            else
            {
                drive.motorPowers[i] = (int)(powers[i]*MOTOR_MAX_VALUE);
            }
        }
    }
    else
    {
        //
        // Mecanum drive is only possible with 4 motors. For 2 motors, we
        // do arcade drive instead.
        //
        DriveArcade(drive, mag, rot);
    }

    drive.flags |= DRIVEF_ON;

    TExit();
    return;
}   //DriveMecanumPolar

/**
 *  This function performs the driving task according to the drive state.
 *
 *  @param drive Points to the DRIVE structure.
 */
void
DriveTask(
    DRIVE &drive
    )
{
    TFuncName("DriveTask");
    TLevel(TASK);
    TEnter();

    if (drive.flags & DRIVEF_ON)
    {
        unsigned long currTime = GetMsecTime();
        float dt = (currTime - drive.prevTime)/1000.0;
        long flPos = nMotorEncoder[drive.motorIDs[IDX_FRONT_LEFT]];
        long frPos = nMotorEncoder[drive.motorIDs[IDX_FRONT_RIGHT]];
        long rlPos = 0;
        long rrPos = 0;
        long flDelta = flPos - drive.motorEncoders[IDX_FRONT_LEFT];
        long frDelta = frPos - drive.motorEncoders[IDX_FRONT_RIGHT];
        long rlDelta = 0;
        long rrDelta = 0;
        float flSpeed = (float)flDelta/dt;
        float frSpeed = (float)frDelta/dt;
        float rlSpeed = 0.0;
        float rrSpeed = 0.0;

        if (abs(flSpeed - drive.motorSpeeds[IDX_FRONT_LEFT]) > 10000.0)
        {
            //
            // Bogus encoder value, ignore it and assume previous speed.
            //
#ifdef _DEBUG_DRIVE
            TPrintfLine("Bogus flSpeed: %6.1f", flSpeed);
#endif
            flSpeed = drive.motorSpeeds[IDX_FRONT_LEFT];
            flDelta = (long)(flSpeed*dt);
            flPos = drive.motorEncoders[IDX_FRONT_LEFT] + flDelta;
        }
        else
        {
            drive.motorSpeeds[IDX_FRONT_LEFT] = flSpeed;
        }

        if (abs(frSpeed - drive.motorSpeeds[IDX_FRONT_RIGHT]) > 10000.0)
        {
            //
            // Bogus encoder value, ignore it and assume previous speed.
            //
#ifdef _DEBUG_DRIVE
            TPrintfLine("Bogus frSpeed: %6.1f", frSpeed);
#endif
            frSpeed = drive.motorSpeeds[IDX_FRONT_RIGHT];
            frDelta = (long)(frSpeed*dt);
            frPos = drive.motorEncoders[IDX_FRONT_RIGHT] + frDelta;
        }
        else
        {
            drive.motorSpeeds[IDX_FRONT_RIGHT] = frSpeed;
        }

        if (DriveHas4Motors(drive))
        {
            rlPos = nMotorEncoder[drive.motorIDs[IDX_REAR_LEFT]];
            rrPos = nMotorEncoder[drive.motorIDs[IDX_REAR_RIGHT]];
            rlDelta = rlPos - drive.motorEncoders[IDX_REAR_LEFT];
            rrDelta = rrPos - drive.motorEncoders[IDX_REAR_RIGHT];
            rlSpeed = (float)rlDelta/dt;
            rrSpeed = (float)rrDelta/dt;

            if (abs(rlSpeed - drive.motorSpeeds[IDX_REAR_LEFT]) > 10000.0)
            {
                //
                // Bogus encoder value, ignore it and assume previous speed.
                //
#ifdef _DEBUG_DRIVE
                TPrintfLine("Bogus rlSpeed: %6.1f", rlSpeed);
#endif
                rlSpeed = drive.motorSpeeds[IDX_REAR_LEFT];
                rlDelta = (long)(rlSpeed*dt);
                rlPos = drive.motorEncoders[IDX_REAR_LEFT] + rlDelta;
            }
            else
            {
                drive.motorSpeeds[IDX_REAR_LEFT] = rlSpeed;
            }

            if (abs(rrSpeed - drive.motorSpeeds[IDX_REAR_RIGHT]) > 10000.0)
            {
                //
                // Bogus encoder value, ignore it and assume previous speed.
                //
#ifdef _DEBUG_DRIVE
                TPrintfLine("Bogus rrSpeed: %6.1f", rrSpeed);
#endif
                rrSpeed = drive.motorSpeeds[IDX_REAR_RIGHT];
                rrDelta = (long)(rrSpeed*dt);
                rrPos = drive.motorEncoders[IDX_REAR_RIGHT] + rrDelta;
            }
            else
            {
                drive.motorSpeeds[IDX_REAR_RIGHT] = rrSpeed;
            }

            if (!DriveIsSwerve(drive))
            {
                //
                // Macanum drive
                //
                drive.xPos += ((flDelta + rrDelta) - (frDelta + rlDelta))/4.0;
                drive.yPos += (flDelta + frDelta + rlDelta + rrDelta)/4.0;
                drive.rotPos += ((flDelta + rlDelta) - (frDelta + rrDelta))/4.0;
            }
#ifdef _SERVO_H
            else
            {
                //
                // Swerve drive.
                //
                float flAngle = ServoGetAngle(*drive.servos[IDX_FRONT_LEFT]);
                float rlAngle = ServoGetAngle(*drive.servos[IDX_REAR_LEFT]);
                float frAngle = ServoGetAngle(*drive.servos[IDX_FRONT_RIGHT]);
                float rrAngle = ServoGetAngle(*drive.servos[IDX_REAR_RIGHT]);
                bool fTranslating = flAngle == frAngle &&
                                    rlAngle == rrAngle &&
                                    flAngle == rrAngle;
                bool fInPlaceTurn = !fTranslating &&
                                    flAngle == rrAngle &&
                                    frAngle == rlAngle;
                float angle = ((flAngle + rlAngle + frAngle + rrAngle)/4.0)*
                              PI/180.0;

                if (fTranslating)
                {
                    float delta = (flDelta + frDelta + rlDelta + rrDelta)/4.0;

                    drive.xPos += delta*sin(angle);
                    drive.yPos += delta*cos(angle);
                }
                else if (fInPlaceTurn)
                {
                    drive.rotPos += ((flDelta + rlDelta) - (frDelta + rrDelta))/
                                    4.0;
                }
                else
                {
                    //???
                    drive.rotPos += (flDelta + frDelta + rlDelta + rrDelta)/4.0;
                }
            }
#endif
        }
        else
        {
            drive.yPos += (flDelta + frDelta)/2.0;
            drive.rotPos += (flDelta - frDelta)/2.0;
        }

#ifdef _DEBUG_DRIVE
        TPrintfLine("dT=%d", currTime - drive.prevTime);
        TPrintfLine("flPower=%4d, rlPower=%4d, frPower=%4d, rrPower=%4d",
                    drive.motorPowers[IDX_FRONT_LEFT],
                    drive.motorPowers[IDX_REAR_LEFT],
                    drive.motorPowers[IDX_FRONT_RIGHT],
                    drive.motorPowers[IDX_REAR_RIGHT]);
        TPrintfLine("flPos=%6d, rlPos=%6d, frPos=%6d, rrPos=%6d",
                    flPos, rlPos, frPos, rrPos);
        TPrintfLine("flSpeed=%6.1f, rlSpeed=%6.1f, frSpeed=%6.1f, rrSpeed=%6.1f",
                    drive.motorSpeeds[IDX_FRONT_LEFT],
                    drive.motorSpeeds[IDX_REAR_LEFT],
                    drive.motorSpeeds[IDX_FRONT_RIGHT],
                    drive.motorSpeeds[IDX_REAR_RIGHT]);
        TPrintfLine("xPos=%6.1f, yPos=%6.1f, rotPos=%6.1f\n",
                    drive.xPos, drive.yPos, drive.rotPos);
#endif

        if ((drive.flags & DRIVEF_STALLED) == 0)
        {
            motor[drive.motorIDs[IDX_FRONT_LEFT]] =
                drive.motorPowers[IDX_FRONT_LEFT];
            motor[drive.motorIDs[IDX_FRONT_RIGHT]] =
                drive.motorPowers[IDX_FRONT_RIGHT];
            if (DriveHas4Motors(drive))
            {
                motor[drive.motorIDs[IDX_REAR_LEFT]] =
                    drive.motorPowers[IDX_REAR_LEFT];
                motor[drive.motorIDs[IDX_REAR_RIGHT]] =
                    drive.motorPowers[IDX_REAR_RIGHT];
            }

            if (drive.flags & DRIVEF_STALL_PROTECT_ON)
            {
                unsigned long currTime = nPgmTime;
                //
                // Check for motor stall conditions:
                // - Stall timer is set AND
                // - Any motors are powered above MIN_STALL_POWER AND
                // - All motors have not moved
                //
                if ((drive.stallTimer == 0) ||
                    (abs(drive.motorPowers[IDX_FRONT_LEFT]) <=
                     DRIVE_MIN_STALL_POWER) &&
                    (abs(drive.motorPowers[IDX_FRONT_RIGHT]) <=
                     DRIVE_MIN_STALL_POWER) &&
                    (!DriveHas4Motors(drive) ||
                     (abs(drive.motorPowers[IDX_REAR_LEFT]) <=
                      DRIVE_MIN_STALL_POWER) &&
                     (abs(drive.motorPowers[IDX_REAR_RIGHT]) <=
                      DRIVE_MIN_STALL_POWER)) ||
                    (flPos != drive.motorEncoders[IDX_FRONT_LEFT]) ||
                    (frPos != drive.motorEncoders[IDX_FRONT_RIGHT]) ||
                    DriveHas4Motors(drive) &&
                    ((rlPos != drive.motorEncoders[IDX_REAR_LEFT]) ||
                     (rrPos != drive.motorEncoders[IDX_REAR_RIGHT])))
                {
                    //
                    // We are not in a stalled situation if any of the
                    // following are true:
                    // - motor powers are below min stall power
                    // - motor encoders showed that they have moved
                    //
                    drive.stallTimer = currTime;
                }

                if (currTime - drive.stallTimer >= DRIVE_STALL_TIME)
                {
                    //
                    // We have detected a stalled condition for at
                    // DRIVE_STALL_TIME.
                    // Let's kill the power of all the motors.
                    //
                    motor[drive.motorIDs[IDX_FRONT_LEFT]] = 0;
                    motor[drive.motorIDs[IDX_FRONT_RIGHT]] = 0;
                    if (DriveHas4Motors(drive))
                    {
                        motor[drive.motorIDs[IDX_REAR_LEFT]] = 0;
                        motor[drive.motorIDs[IDX_REAR_RIGHT]] = 0;
                    }
                    drive.flags |= DRIVEF_STALLED;
                    PlayImmediateTone(1000, 100);
                }
            }
        }

        drive.prevTime = currTime;
        drive.motorEncoders[IDX_FRONT_LEFT] = flPos;
        drive.motorEncoders[IDX_FRONT_RIGHT] = frPos;
        if (DriveHas4Motors(drive))
        {
            drive.motorEncoders[IDX_REAR_LEFT] = rlPos;
            drive.motorEncoders[IDX_REAR_RIGHT] = rrPos;
        }
    }
    else
    {
        //
        // The motors should be OFF.
        // Let's make sure they are.
        //
        motor[drive.motorIDs[IDX_FRONT_LEFT]] = 0;
        motor[drive.motorIDs[IDX_FRONT_RIGHT]] = 0;
        if (DriveHas4Motors(drive))
        {
            motor[drive.motorIDs[IDX_REAR_LEFT]] = 0;
            motor[drive.motorIDs[IDX_REAR_RIGHT]] = 0;
        }
    }

    TExit();
    return;
}   //DriveTask

#endif  //ifndef _DRIVE_H
