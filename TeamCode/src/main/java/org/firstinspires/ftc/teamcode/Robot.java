package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessorImpl;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class Robot {

    // Attributes for hardware
    DriveTrain driveTrain = null;
    private Lift lift = null;
    private final double liftPowerFactor = 0.7;   // Default of 0.7
    private Rev2mDistanceSensor distanceSensor = null;
    private Servo wrist = null;
    private double wristDownPosition = 0;
    private double wristUpPosition = 0.7;
    private Servo grabber = null;
    private double grabberClosedPosition = 0.3;
    private double grabberOpenPosition = 0.7;

    CameraName camera = null;
    private OpMode opMode = null;

    public Robot(OpMode opMode) {
        this.opMode = opMode;

        driveTrain = new DriveTrain(opMode);

        lift = new Lift(opMode, liftPowerFactor);

        wrist = opMode.hardwareMap.get(Servo.class, "wrist");
        wrist.setPosition(wristDownPosition);

        grabber = opMode.hardwareMap.get(Servo.class, "grabber");
        grabber.setPosition(grabberClosedPosition);

        camera = opMode.hardwareMap.get(WebcamName.class, "Webcam 1");
    }

    public void autoDriveToAprilTag(int targetTagNumber, double targetDistance) {
        AprilTagDetection targetTag = null;
        boolean targetFound = false;
        double axialError = 0;
        double strafeError = 0;
        double yawError = 0;
        double axialErrorThreshold = 0.5;
        double axialPower = 0;
        double strafePower = 0;
        double yawPower = 0;
        final double axialGain = 0.02;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
        final double strafeGain = 0.015;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
        final double yawGain = 0.01;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
        final double maxAxial = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
        final double maxStrafe = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
        final double maxYaw = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)


        AprilTagProcessor aprilTagProcessor = createAprilTagProcessor();
        VisionPortal visionPortal = createVisionPortal(aprilTagProcessor);

        while (axialError > axialErrorThreshold) {
            targetTag = null;
            targetFound = false;
            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((targetTagNumber < 0) || (detection.id == targetTagNumber)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        targetTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        opMode.telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    opMode.telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                opMode.telemetry.addData("\n>", "HOLD Left-Bumper to Drive to Target\n");
                opMode.telemetry.addData("Found", "ID %d (%s)", targetTag.id, targetTag.metadata.name);
                opMode.telemetry.addData("Range", "%5.1f inches", targetTag.ftcPose.range);
                opMode.telemetry.addData("Bearing", "%3.0f degrees", targetTag.ftcPose.bearing);
                opMode.telemetry.addData("Yaw", "%3.0f degrees", targetTag.ftcPose.yaw);
            } else {
                opMode.telemetry.addData("\n>", "Drive until bot finds valid target\n");
            }

            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            axialError = (targetTag.ftcPose.range - targetDistance);
            strafeError = targetTag.ftcPose.bearing;
            yawError = targetTag.ftcPose.yaw;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            axialPower = Range.clip(axialError * axialGain, -maxAxial, maxAxial);
            strafePower = Range.clip(strafeError * strafeGain, -maxStrafe, maxStrafe);
            yawPower = Range.clip(-yawError * yawGain, -maxYaw, maxYaw);

            opMode.telemetry.addData("Auto", "Axial %5.2f, Strafe %5.2f, Yaw %5.2f ", axialPower, strafePower, yawPower);

            opMode.telemetry.update();

            // Apply desired axes motions to the drivetrain.
            moveDirection(axialPower, strafePower, yawPower);
           // sleep(10);
        }
    }

    // Scan for prop in one of three positions
    // Return position 1, 2, or 3 (left, right, or center)
    public byte determineObjectPosition() {
        return 1;
    }

    // Turn to a specified heading in degrees
    // 0 is straight ahead
    // < 0 is CCW
    // > 0 is CW
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

    // Move straight for a specified distance in inches
    public void moveStraightForDistance(double distance) {
        driveTrain.moveStraightForDistance(distance);
    }

    public void moveStraightToObject() {
        driveTrain.moveStraightForDistance(distanceSensor.getDistance(DistanceUnit.INCH));
    }

    private AprilTagProcessor createAprilTagProcessor() {
        // Create the AprilTag processor by using a builder.
        AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTagProcessor.setDecimation(1);

        return aprilTagProcessor;
    }

    private VisionPortal createVisionPortal(AprilTagProcessor aprilTagProcessor) {
        VisionPortal visionPortal = new VisionPortal.Builder().setCamera(camera)
                .addProcessor(aprilTagProcessor).build();
        return visionPortal;
    }
}


