package org.firstinspires.ftc.teamcode;

public class RevMotorInfo {

    static final double     COUNTS_PER_MOTOR_REV    = 28 * 4;    // eg: REV HD Hex Motor
    static final double     DRIVE_GEAR_REDUCTION    = 5.23 * 2.89;     // REV 5:1 and 3:1 UltraPl
    static final double     WHEEL_DIAMETER_MM       = 4.0;     // For figuring circumference
    static final double     COUNTS_PER_MM           = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_MM * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    static final double MAX_MOTOR_RPS               = 100;

    static final double MAX_VELOCITY                = MAX_MOTOR_RPS * COUNTS_PER_MOTOR_REV /
            DRIVE_GEAR_REDUCTION;
}
