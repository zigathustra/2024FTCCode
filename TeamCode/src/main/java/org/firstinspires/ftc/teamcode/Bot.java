package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessorImpl;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class Bot {
    // Attributes for hardware
    protected DriveTrain driveTrain = null;
    protected Lift lift = null;
    protected Servo wrist = null;
    protected Servo grabber = null;
    protected Rev2mDistanceSensor distanceSensor = null;
    protected LinearOpMode opMode = null;
    protected double maxSpeed = Constants.maxNormalSpeed; // Default speed. Reassigned in the constructor.

    public Bot(LinearOpMode opMode, double maxSpeed) {
        this.opMode = opMode;
        this.maxSpeed = maxSpeed;

        driveTrain = new DriveTrain(opMode, maxSpeed);

        lift = new Lift(opMode);

        wrist = opMode.hardwareMap.get(Servo.class, "wrist");
        wrist.setPosition(Constants.wristDownPosition);

        grabber = opMode.hardwareMap.get(Servo.class, "grabber");
        grabber.setPosition(Constants.grabberClosedPosition);

        distanceSensor = opMode.hardwareMap.get(Rev2mDistanceSensor.class, "distance_sensor");
    }

    public void setRunWithoutEncoders() {
        driveTrain.setRunWithoutEncoders();
    }

    // Turn to a specified heading in degrees
    // 0 is straight ahead
    // > 0 is CCW
    // < 0 is CW
    public void turnToHeading(double heading) {
        driveTrain.turnToHeading(heading);
    }

    // Turn a specified distance in degrees
    public void turnForDistance(double distance) {
        driveTrain.turnForDistance(distance);
    }

    public void moveDirection(double axial, double strafe, double yaw) {
        driveTrain.moveDirection(axial, strafe, yaw);
    }

    public void moveDirectionNoEnc(double axial, double strafe, double yaw) {
        driveTrain.moveDirection(axial, strafe, yaw);
    }

    public void creepDirection(double axial, double strafe, double yaw) {
        driveTrain.creepDirection(axial, strafe, yaw);
    }

    public void creepDirectionNoEnc(double axial, double strafe, double yaw) {
        driveTrain.creepDirectionNoEnc(axial, strafe, yaw);
    }

    public void creepStraightForDistance(double distance) {
        driveTrain.creepStraightForDistance(distance);
    }

    public void creepUntilContact() {
    }

    // Move straight for a specified distance in inches
    public void moveStraightForDistance(double distance) {
        driveTrain.moveStraightForDistance(distance);
    }

    public void strafeForDistance(double distance) {
        driveTrain.strafeForDistance(distance);
    }

    public void stopDrive() {
        driveTrain.moveDirection(0, 0, 0);
    }

    public void liftUp(double speed) {
        lift.liftUp(speed);
    }

    public void liftDown(double speed) {
        lift.liftDown(speed);
    }

    public void liftStop() {
        lift.stop();
    }

    public void liftStopAtPosition(int position) {
        lift.stopAtPosition(position);
    }

    public void wristUp() {
        wrist.setPosition(Constants.wristUpPosition);
    }

    public void wristMiddle() {
        wrist.setPosition(Constants.wristMiddlePosition);
    }

    public void wristDown() {
        wrist.setPosition(Constants.wristDownPosition);
    }

    public void grabberOpen() {
        grabber.setPosition(Constants.grabberOpenPosition);
    }

    public void grabberClose() {
        grabber.setPosition(Constants.grabberClosedPosition);
    }

    public double getDistance() {
        double distance;
        opMode.sleep(50);
        distance = distanceSensor.getDistance(DistanceUnit.INCH);
        opMode.sleep(50);
        return (distance);
    }
}


