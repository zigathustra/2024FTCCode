package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveTrain {
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx leftRearDrive = null;
    private DcMotorEx rightRearDrive = null;
    private final double turnGain = 0.02;   // Larger is more responsive, but also less stable
    private final double driveGain = 0.03;
    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    private final double maxAutoDriveSpeed = 0.4;     // Max driving speed for better distance accuracy.
    private final double maxAutoTurnSpeed = 0.2;// Max Turn speed to limit turn rate
    private final double headingThreshold = 1.0;   // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    private final double powerFactor = 0.7;
    private final double maxVelocity = RevUltra20DcMotorData.maxCountsPerSec;
    private final double countsPerInch = RevUltra20DcMotorData.countsPerInch;
    private IMU imu = null;
    OpMode opMode = null;

    public DriveTrain(OpMode opMode) {

        this.opMode = opMode;
        leftFrontDrive = opMode.hardwareMap.get(DcMotorEx.class, "left_front_drive");
        leftRearDrive = opMode.hardwareMap.get(DcMotorEx.class, "left_rear_drive");
        rightFrontDrive = opMode.hardwareMap.get(DcMotorEx.class, "right_front_drive");
        rightRearDrive = opMode.hardwareMap.get(DcMotorEx.class, "right_rear_drive");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);

        stopAndResetEncoders();
        setRunUsingEncoder();
        setBrakingOn();

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = opMode.hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    public void turnToHeading(double targetHeading) {
        double turnSpeed = maxAutoTurnSpeed;
        double headingError = getHeadingError(targetHeading);

        // keep looping while we are still active, and not on heading.
        while (Math.abs(headingError) > headingThreshold){
            headingError = getHeadingError(targetHeading);

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(headingError, turnGain);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxAutoTurnSpeed, maxAutoTurnSpeed);

            // Pivot in place by applying the turning correction
            moveDirection(0, 0, turnSpeed);
        }

        stop();
    }

    public void turnForDistance(double distance) {
        double targetHeading = getHeading() + distance;
        turnToHeading(targetHeading);
    }

    public void moveDirection(double axial, double strafe, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower = axial - strafe - yaw;
        double rightFrontPower = axial + strafe + yaw;
        double leftRearPower = axial + strafe - yaw;
        double rightRearPower = axial + strafe + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftRearPower));
        max = Math.max(max, Math.abs(rightRearPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftRearPower /= max;
            rightRearPower /= max;
        }

        leftFrontDrive.setVelocity(leftFrontPower * powerFactor * maxVelocity);
        rightFrontDrive.setVelocity(rightFrontPower * powerFactor * maxVelocity);
        leftRearDrive.setVelocity(leftRearPower * powerFactor * maxVelocity);
        rightRearDrive.setVelocity(rightRearPower * powerFactor * maxVelocity);
    }

    public void moveStraightForDistance(double distance) {
        int targetCounts = (int) (distance * countsPerInch);
        int leftFrontTarget = 0;
        int leftRearTarget = 0;
        int rightFrontTarget = 0;
        int rightRearTarget = 0;
        double headingError = 0;
        double turnSpeed = maxAutoTurnSpeed;
        double driveSpeed = maxAutoDriveSpeed;
        double targetHeading = getHeading();

        leftFrontTarget = leftFrontDrive.getCurrentPosition() + targetCounts;
        leftRearTarget = leftRearDrive.getCurrentPosition() + targetCounts;
        rightFrontTarget = rightFrontDrive.getCurrentPosition() + targetCounts;
        rightRearTarget = rightRearDrive.getCurrentPosition() + targetCounts;

        leftFrontDrive.setTargetPosition(leftFrontTarget);
        leftRearDrive.setTargetPosition(leftRearTarget);
        rightFrontDrive.setTargetPosition(rightFrontTarget);
        rightRearDrive.setTargetPosition(rightRearTarget);

        setRunToPosition();

        // Set the required driving speed  (must be positive for RUN_TO_POSITION)
        // Start driving straight, and then enter the control loop
        moveDirection(driveSpeed, 0, 0);

        // keep looping while we are still active, and BOTH motors are running.
        while (leftFrontDrive.isBusy() && leftRearDrive.isBusy() && rightFrontDrive.isBusy() && rightRearDrive.isBusy()) {
            headingError = getHeadingError(targetHeading);
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(headingError, driveGain);

            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0)
                turnSpeed *= -1.0;

            // Apply the turning correction to the current driving speed.
            moveDirection(driveSpeed, 0.0, turnSpeed);
        }

        stop();
        setRunUsingEncoder();
    }

    public double getHeadingError(double targetHeading) {
        return (targetHeading - getHeading());
    }

    public double getSteeringCorrection(double headingError, double gain) {
        // Determine the heading current error

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * gain, -1, 1);
    }

    private double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    private void stop() {
        leftFrontDrive.setVelocity(0.0);
        rightFrontDrive.setVelocity(0.0);
        leftRearDrive.setVelocity(0.0);
        rightRearDrive.setVelocity(0.0);
    }

    private void setBrakingOn() {
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void setBrakingOff() {
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    private void stopAndResetEncoders() {
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void setRunUsingEncoder() {
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setRunToPosition() {
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
}
