package org.firstinspires.ftc.teamcode;

public class Constants {
    static final double driveTrainMaxVelocity = RevUltra20DcMotorData.maxCountsPerSec;
    static final double driveTrainCountsPerInch = RevUltra20DcMotorData.countsPerInch;
    static final double liftMaxVelocity = GoBilda312DcMotorData.maxCountsPerSec;
    static final double maxNormalSpeed = 0.8;
    static final double maxCreepSpeed = 0.15;
    static final double maxAutoSpeed = 0.4;
    static final double atAxialGain = 0.02;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    static final double atStrafeGain = 0.01;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    static final double atYawGain = 0.01;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    static final double atMaxAxial = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    static final double atMaxStrafe = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    static final double atMaxYaw = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    static final int atExposureMS = 6;
    static final int atExposureGain = 250;
    static final double dsPropDistanceThreshold = 12;
    static final int dsPosition1Heading = 90;
    static final int dsPosition2Heading = 0;
    static final int dsPosition3Heading = -90;
    static final double dsPlacementDistanceOffset = 5.5;
    static final double dsPosition1StrafeDistance = 4;
    static final double dsPosition2StrafeDistance = 3.5;
    static final double dsPosition3StrafeDistance = 12;
    static final double autoTurnGain = 0.02;   // Larger is more responsive, but also less stable
    static final double autoDriveGain = 0.03;
    static final double maxAutoStrafeSpeed = 0.4;
    static final double maxAutoCorrectionDriveSpeed = 0.4; // Max driving speed for better distance accuracy
    static final double maxAutoCorrectionTurnSpeed = 0.4;// Max Turn speed to limit turn rate
    static final double autoHeadingThreshold = 1.0;   // How close the heading must be to the target
    static final double liftMaxMoveSpeed = 1.0;
    static final double liftStopPowerFactor = 1.0;
    static final int liftMaxPosition = 3000;
    static final int liftMaxTolerance = 25;
    static final int liftMinPosition = 0;
    static final int liftminTolerance = 25;
}
