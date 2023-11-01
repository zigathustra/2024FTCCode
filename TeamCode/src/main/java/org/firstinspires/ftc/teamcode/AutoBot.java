package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class AutoBot extends Bot {
    private AprilTagProcessor aprilTagProcessor = null;
    private VisionPortal visionPortal = null;
    protected Rev2mDistanceSensor distanceSensor = null;

    public AutoBot(LinearOpMode opMode) {
        super(opMode, Constants.maxAutoSpeed);
        distanceSensor = opMode.hardwareMap.get(Rev2mDistanceSensor.class, "distance_sensor");
    }

    public void autoDriveToAprilTag(int targetTagNumber, double targetDistanceFromTag) {
        AprilTagDetection targetTag = null;
        boolean targetFound = false;
        double axialError = 1000;
        double strafeError = 1000;
        double yawError = 1000;
        double axialErrorThreshold = 0.5;
        double axialPower = 0;
        double strafePower = 0;
        double yawPower = 0;
        double scanAngle = 2;
        final double maxTotalAngle = 15;
        final double minTotalAngle = -15;
        double totalAngle = 0;

        initAprilTag();

        setManualExposure(Constants.atExposureMS, Constants.atExposureGain);  // Use low exposure time to reduce motion blur

        while (axialError > axialErrorThreshold) {
            targetFound = false;
            targetTag = null;

            // Search through detected tags to find the target tag
            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag
                    if (detection.id == targetTagNumber) {
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
                opMode.telemetry.addData("Found", "ID %d (%s)", targetTag.id, targetTag.metadata.name);
                opMode.telemetry.addData("Range", "%5.1f inches", targetTag.ftcPose.range);
                opMode.telemetry.addData("Bearing", "%3.0f degrees", targetTag.ftcPose.bearing);
                opMode.telemetry.addData("Yaw", "%3.0f degrees", targetTag.ftcPose.yaw);
            }

            if (targetFound) {
                totalAngle = 0;
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                axialError = (targetTag.ftcPose.range - targetDistanceFromTag);
                strafeError = targetTag.ftcPose.bearing;
                yawError = targetTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                axialPower = Range.clip(axialError * Constants.atAxialGain, -Constants.atMaxAxial, Constants.atMaxAxial);
                yawPower = Range.clip(strafeError * Constants.atYawGain, -Constants.atMaxYaw, Constants.atMaxYaw);
                strafePower = Range.clip(-yawError * Constants.atStrafeGain, -Constants.atMaxStrafe, Constants.atMaxStrafe);

                opMode.telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", axialPower, strafePower, yawPower);
            } else {
                // scan for tags
                if ((totalAngle >= maxTotalAngle) || (totalAngle <= minTotalAngle)) {
                    scanAngle = -scanAngle;
                }
                totalAngle = totalAngle + scanAngle;
                turnForDistance(scanAngle);
                axialPower = 0.0;
                strafePower = 0.0;
                yawPower = 0.0;
                opMode.telemetry.addData("Scanning Angle:", "%4.12f", scanAngle);
            }
            opMode.telemetry.update();

            // Apply desired axes motions to the drivetrain.
            driveTrain.moveDirection(axialPower, strafePower, yawPower);
            opMode.sleep(10);
        }
    }

    public double getDistance() {
        return distanceSensor.getDistance(DistanceUnit.INCH);
    }

    public void moveStraightToObject(double targetDistance) {
//        double distance = 12;
        double objectDistance = distanceSensor.getDistance(DistanceUnit.INCH);
        opMode.telemetry.addData("Distance: ", objectDistance);
        opMode.telemetry.update();
        opMode.sleep(3000);
//        if (objectDistance < targetDistance) {
//            distance = objectDistance;
//        } else {
//            distance = objectDistance - targetDistance;
//        }
        moveStraightForDistance(objectDistance-targetDistance);
    }

    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTagProcessor = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTagProcessor.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .build();
    }

    //   Manually set the camera gain and exposure.
    //   This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls
        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            opMode.telemetry.addData("Camera", "Waiting");
            opMode.telemetry.update();
            while (!opMode.isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                opMode.sleep(20);
            }
            opMode.telemetry.addData("Camera", "Ready");
            opMode.telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!opMode.isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                opMode.sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            opMode.sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            opMode.sleep(20);
        }
    }
}
