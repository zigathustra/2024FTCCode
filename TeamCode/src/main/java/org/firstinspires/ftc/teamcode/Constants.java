package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Constants {
    // Drivetrain Settings
    static final double drivetrainLength = 14;
    static final double distanceBetweenSensors = 8.75;
    static double sensorToDrivetrainMiddle = distanceBetweenSensors/2; // Distance from distance sensor to the middle of the drivetrain
    static double cameraToDrivetrainMiddle = distanceBetweenSensors/2; // Distance from camera to the middle of the drivetrain
    static final double driveTrainMaxVelocity = GoBilda312DcMotorData.maxCountsPerSec;
    static final double mecanumMoveFactor = .95;
    static final double mecanumMoveCountsPerInch = mecanumMoveFactor * GoBilda312DcMotorData.wheelCountsPerInch;

    static final double mecanumStrafeFactor = 1.075;
    static final double mecanumStrafeCountsPerInch = mecanumStrafeFactor * GoBilda312DcMotorData.wheelCountsPerInch;
    static final DcMotor.Direction drivetrainLeftFrontDirection = DcMotor.Direction.REVERSE;
    static final DcMotor.Direction drivetrainLeftRearDirection = DcMotor.Direction.REVERSE;
    static final DcMotor.Direction drivetrainRightFrontDirection = DcMotor.Direction.FORWARD;
    static final DcMotor.Direction drivetrainRightRearDirection = DcMotor.Direction.FORWARD;
    static final double maxNormalSpeed = 0.8;
    static final double maxCreepSpeed = 0.15;
    static final double maxAutoSpeed = 0.5;
    static final double autoTurnGain = 0.02;   // Larger is more responsive, but also less stable
    static final double autoDriveGain = 0.03;
    static final double maxAutoStrafeSpeed = 0.5;
    static final double maxAutoCorrectionDriveSpeed = 0.5; // Max driving speed for better distance accuracy
    static final double maxAutoCorrectionTurnSpeed = 0.5; // Max Turn speed to limit turn rate
    static final double autoHeadingThreshold = 0.5; // How close the heading must be to the target

    // Lift Settings
    static final double liftMaxVelocity = GoBilda312DcMotorData.maxCountsPerSec;
    static final double liftMaxMoveSpeed = 1.0;
    static final double liftStopPowerFactor = 1.0;
    static final int liftMaxPosition = 3000;
    static final int liftMaxTolerance = 25;
    static final int liftMinPosition = 0;
    static final int liftMinTolerance = 25;
    static final int liftAutoBoardPosition = 950;
    static final int liftAutoCruisingPosition = 500;

    // Wrist settings
    static final double wristDownPosition = 0.4;
    static final double wristMiddlePosition = 0.5;
    static final double wristUpPosition = 0.95;
    // Grabber settings
    static final double grabberClosedPosition = 0.45;
    static final double grabberOpenPosition = 1.5;

    // Prop Detection Settings
    static final double dsDistanceToCenterOfSpikeMarks = 25.0;
    static final double dsPropDistanceThreshold = 14;

    static final double dsPlacementDistanceOffset = 5.5;
    static final double dsFarPositionStrafeDistance = 8;
    static final double dsMiddlePositionStrafeDistance = 4.5;
    static final double dsNearPositionStrafeDistance = 12;

    // April Tag Detection Settings
    static final double atAxialGain = 0.02;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    static final double atStrafeGain = 0.02;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    static final double atYawGain = 0.02;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    static final double atMaxAxial = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    static final double atMaxStrafe = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    static final double atMaxYaw = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    static final int atExposureMS = 6;
    static final int atExposureGain = 250;

    // Pixel placement settings
    static final double boardApproachDistance = 4;
    static final double boardOffsetDistance = 8;
    static final double boardEscapeDistance = 10;

    // Parking Settings
    static final double distanceBetweenAprilTags = 6;
}
