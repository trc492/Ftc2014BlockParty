#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="RobotInfo.h" />
///
/// <summary>
///     This module contains the Robot Info constants.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _ROBOTINFO_H
#define _ROBOTINFO_H

//
// Drive subsystem info.
//
#define SERVO_MIN_VALUE         50      //50
#define SERVO_MAX_VALUE         185     //185
#define SERVO_MAX_ANGLE         180.0   //180.0
#define SERVO_MAX_STEP_RATE     180.0   //180.0
#define SERVO_FL_ZERO_ANGLE     90.0    //90.0
#define SERVO_FR_ZERO_ANGLE     90.0    //90.0
#define SERVO_RL_ZERO_ANGLE     90.0    //90.0
#define SERVO_RR_ZERO_ANGLE     90.0    //90.0

#define SWERVE_DELAY            1000    //msec

#define CLICKS_PER_XUNITS       151.818 //1440/(3*PI)
#define CLICKS_PER_YUNITS       151.818 //1440/(3*PI)
#define CLICKS_PER_ROTUNITS     20.667  //1440/(3*PI*360/(15.5*PI))

#define DRIVE_MIN_VALUE         -75
#define DRIVE_MAX_VALUE         75
#define TURN_MIN_VALUE          -70
#define TURN_MAX_VALUE          70

#define ENCX_KP                 3.3     //3.3
#define ENCX_KI                 0.0     //0.0
#define ENCX_KD                 0.0     //0.0
#define ENCX_TOLERANCE          1.0
#define ENCX_SETTLING           200

#define ENCY_KP                 3.3     //3.3
#define ENCY_KI                 0.0     //0.0
#define ENCY_KD                 0.0     //0.0
#define ENCY_TOLERANCE          1.0
#define ENCY_SETTLING           200

#define GYROROT_KP              2.3     //2.5
#define GYROROT_KI              0.0     //0.0
#define GYROROT_KD              2.8     //2.8
#define GYROROT_TOLERANCE       1.0
#define GYROROT_SETTLING        200

#define GYROTURN_KP             5.0     //5.0
#define GYROTURN_KI             0.0     //0.0
#define GYROTURN_KD             0.0     //0.0
#define GYROTURN_TOLERANCE      1.0
#define GYROTURN_SETTLING       200

#define IR_KP                   10.0    //10.0
#define IR_KI                   0.0
#define IR_KD                   3.0
#define IR_TOLERANCE            1.0
#define IR_SETTLING             200
#define IR_TARGET               10.0

#define SONAR_KP                2.6     //2.6
#define SONAR_KI                0.0
#define SONAR_KD                0.0
#define SONAR_TOLERANCE         1.0
#define SONAR_SETTLING          200
#define SONAR_TARGET            10.0    //14.0      inches
#define SONAR_PENDULUM_THRESHOLD 36.0

#define COLOR_KP                10.0
#define COLOR_KI                0.0
#define COLOR_KD                0.0
#define COLOR_TOLERANCE         1.0
#define COLOR_SETTLING          200

#define COLOR_BLACK             12
#define COLOR_WHITE             17
#define COLOR_RED               9
#define COLOR_BLUE              2
#define COLOR_WHITE_THRESHOLD   150     //((30+240)/2)

#define ACCEL_KP                50.0
#define ACCEL_KI                0.0
#define ACCEL_KD                0.0
#define ACCEL_TOLERANCE         1.0
#define ACCEL_SETTLING          200
#define ACCEL_SLOPE_TARGET      -0.4

//
// Slide subsystem info.
//
#define SLIDE_KP                500.0   //500.0
#define SLIDE_KI                0.0
#define SLIDE_KD                0.0
#define SLIDE_TOLERANCE         1.0     //inches
#define SLIDE_SETTLING          200     //msec

#define SLIDE_CLICKS_PER_INCH   3840.0  //(1440*4*16.0/24.0)
#define SLIDE_CAL_POWER         -50
#define SLIDE_STALL_MINPOWER    30
#define SLIDE_STALL_TIMEOUT     100     //msec
#define SLIDE_UPPER_LIMIT       8.5     //9.0
#define SLIDE_LOWER_LIMIT       4.5     //0.0

//
// Arm subsystem info.
//
#define ARM_KP                  20.0    //20.0
#define ARM_KI                  0.0
#define ARM_KD                  0.001   //0.001
#define ARM_TOLERANCE           1.0     //degrees
#define ARM_SETTLING            200     //msec

#define ARM_CLICKS_PER_DEGREE   64.0    //(1440*16.0/360.0)
#define ARM_CAL_POWER           -50
#define ARM_STALL_MINPOWER      30
#define ARM_STALL_TIMEOUT       100     //msec
#define ARM_UPPER_LIMIT         92.0    //degrees
#define ARM_LOWER_LIMIT         0.0

#define ARM_SCORE_POS           70.0

//
// Scoop subsystem info.
//
#define SCOOP_DOWN_POS          90.0
#define SCOOP_UP_POS            180.0
#define SCOOP_INC               6.0     //degrees
#define SCOOP_DUMP_TIME         200     //msec

//
// William subsystem info.
//
#define WILLIAM_DOWN_POS        180.0
#define WILLIAM_MID_POS         90.0
#define WILLIAM_UP_POS          0.0

//
// Crank subsystem info.
//
#define CRANK_POWER             100

//
// Autonomous definitions.
//

#define AUTO_INIT_DELAY         0

// Strategies menu.
#define STRATEGY_NOAUTO         0
#define STRATEGY_DEFENSE        1
#define STRATEGY_SIMPLE_SCORE   2
#define STRATEGY_IR_SCORE       3
#define STRATEGY_PARK_ONLY      4

// Park only options.
#define PARK_OPTION_NONE        0
#define PARK_OPTION_LEFT        1
#define PARK_OPTION_RIGHT       2

// IRScore options.
// side|park left:0 right:1
#define IR_OPTION_START_MASK    0x1
#define IR_OPTION_START_LEFT    0x0
#define IR_OPTION_START_RIGHT   0x1
#define IR_OPTION_PARK          0x2
#define IROptionStartRight(o)   (((o) & IR_OPTION_START_MASK) == IR_OPTION_START_RIGHT)
#define IROptionPark(o)         (((o) & IR_OPTION_PARK) != 0)
#define IR_PENDULUM_LEN         52.0

// Park Distances.
#define PARK_DIST_NONE          0
#define PARK_DIST_NEAR          1
#define PARK_DIST_FAR           2

//Joystick enable/disables
//#define DISABLE_SLIDE_ZERO

#endif  //ifndef _ROBOTINFO_H
