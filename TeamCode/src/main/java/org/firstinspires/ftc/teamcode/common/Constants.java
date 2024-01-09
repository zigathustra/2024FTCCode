package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Constants {
    // Drivetrain Settings
    public static final double drivetrainLength = 14;
    public static final double distanceBetweenSensors = 8.75;
    public static double sensorToDrivetrainMiddle = distanceBetweenSensors/2; // Distance from distance sensor to the middle of the drivetrain
    public static double cameraToDrivetrainMiddle = distanceBetweenSensors/2; // Distance from camera to the middle of the drivetrain
    public static final double driveTrainMaxVelocity = GoBilda312DcMotorData.maxCountsPerSec;
    public static final double mecanumMoveFactor = .95;
    public static final double mecanumMoveCountsPerInch = mecanumMoveFactor * GoBilda312DcMotorData.wheelCountsPerInch;
    public static final double mecanumStrafeFactor = 1.075;
    public static final double mecanumStrafeCountsPerInch = mecanumStrafeFactor * GoBilda312DcMotorData.wheelCountsPerInch;
    public static final DcMotor.Direction drivetrainLeftFrontDirection = DcMotor.Direction.REVERSE;
    public static final DcMotor.Direction drivetrainLeftRearDirection = DcMotor.Direction.REVERSE;
    public static final DcMotor.Direction drivetrainRightFrontDirection = DcMotor.Direction.FORWARD;
    public static final DcMotor.Direction drivetrainRightRearDirection = DcMotor.Direction.FORWARD;
    public static final double maxNormalSpeed = 0.8;
    public static final double maxCreepSpeed = 0.15;
    public static final double maxAutoSpeed = 0.4;
    public static final double autoTurnGain = 0.02;   // Larger is more responsive, but also less stable
    public static final double autoDriveGain = 0.03;
    public static final double maxAutoStrafeSpeed = 0.4;
    public static final double maxAutoCorrectionDriveSpeed = 0.5; // Max driving speed for better distance accuracy
    public static final double maxAutoCorrectionTurnSpeed = 0.5; // Max Turn speed to limit turn rate
    public static final double autoHeadingThreshold = 0.5; // How close the heading must be to the target

    // Lift Settings
    public static final double liftMaxVelocity = GoBilda312DcMotorData.maxCountsPerSec;
    public static final double liftMaxMoveSpeed = 1.0;
    public static final double liftStopPowerFactor = 1.0;
    public static final int liftMaxPosition = 3000;
    public static final int liftMaxTolerance = 25;
    public static final int liftMinPosition = 0;
    public static final int liftMinTolerance = 25;
    public static final int liftAutoBoardProbePosition = 1750;
    public static final int liftAutoBoardPlacementPosition = 1100;
    public static final int liftAutoHighCruisingPosition = 550;
    public static final int liftAutoLowCruisingPosition = 100;

    // Wrist settings
    public static final double wristDownPosition = 0.85;
    public static final double wristMiddlePosition = 0.73;
    public static final double wristUpPosition = 0.4;

    // Grabber settings
    public static final double grabberClosedPosition = 0.45;
    public static final double grabberOpenPosition = 1.5;

    // Launcher settings
    public static final double launcherLockedPosition = 1;
    public static final double launcherUnlockedPosition = 0;

    // Prop Detection Settings
    public static final double pdDistanceToEscapePosition = 14;
    public static final double pdLeftPlacementDistance = 10.5;
    public static final double pdLeftHeading = 90;
    public static final double pdCenterPlacementDistance = 24.0;
    public static final double pdCenterHeading = 0;
    public static final double pdRightPlacementDistance = 10.5;
    public static final double pdRightHeading = -90;
    public static final double pdEscapeStrafeDistance = 23;


    // April Tag Detection Settings
    public static final double atAxialGain = 0.02;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    public static final double atStrafeGain = 0.02;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    public static final double atYawGain = 0.02;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    public static final double atMaxAxial = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    public static final double atMaxStrafe = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    public static final double atMaxYaw = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    public static final int atExposureMS = 6;
    public static final int atExposureGain = 250;

    // Pixel placement settings
    public static final double boardApproachDistance = 4;
    public static final double boardOffsetDistance = 2;
    public static final double boardEscapeDistance = 10;

    // Parking Settings
    public static final double distanceBetweenAprilTags = 6;
}
