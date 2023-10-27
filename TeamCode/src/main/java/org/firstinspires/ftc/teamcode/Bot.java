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
    protected double wristDownPosition = 0.4;
    protected double wristUpPosition = 0.75;
    protected Servo grabber = null;
    protected double grabberClosedPosition = 0.4;
    protected double grabberOpenPosition = 1.5;
    protected LinearOpMode opMode = null;

    public Bot(LinearOpMode opMode) {
        this.opMode = opMode;

        driveTrain = new DriveTrain(opMode);

        lift = new Lift(opMode);

        wrist = opMode.hardwareMap.get(Servo.class, "wrist");
        wrist.setPosition(wristDownPosition);

        grabber = opMode.hardwareMap.get(Servo.class, "grabber");
        grabber.setPosition(grabberClosedPosition);

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

    public void creepDirection(double axial, double strafe, double yaw) {
        driveTrain.creepDirection(axial, strafe, yaw);
    }

    // Move straight for a specified distance in inches
    public void moveStraightForDistance(double distance) {
        driveTrain.moveStraightForDistance(distance);
    }

    public void strafeForDistance(double distance) {
        driveTrain.encoderStrafeForDistance(distance);
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
        wrist.setPosition(wristUpPosition);
    }

    public void wristDown() {
        wrist.setPosition(wristDownPosition);
    }

    public void grabberOpen() {
        grabber.setPosition(grabberOpenPosition);
    }

    public void grabberClose() {
        grabber.setPosition(grabberClosedPosition);
    }

    private void logTelemetry() {
    }
}


