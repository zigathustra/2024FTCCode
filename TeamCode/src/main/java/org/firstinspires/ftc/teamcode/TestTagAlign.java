package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "TestTagAlign", group = "Autonomous")
public class TestTagAlign extends LinearOpMode {
    AprilTagDetection targetTag;
    boolean targetFound;
    double strafeError;
    double yawError;
    double strafePower = 1;
    double yawPower = 1;
    double minStrafePower = .01;
    double minYawPower = .01;
    protected Bot bot;
    @Override
    public void runOpMode() {

        int targetAprilTagNumber = 5;

        AprilTagProcessor aprilTagProcessor = null;
        VisionPortal visionPortal = null;
        bot = new Bot(this, Constants.maxAutoSpeed);

        aprilTagProcessor = createAprilTagProcessor();

        visionPortal = createVisionPortal(Constants.atExposureMS, Constants.atExposureGain, aprilTagProcessor);

        bot.liftStopAtPosition(750);

        sleep(1000);

        waitForStart();

        // Correct strafe to directly face the target April Tag
        autoOrientToAprilTag(aprilTagProcessor, targetAprilTagNumber);
    }

    protected void autoOrientToAprilTag(AprilTagProcessor aprilTagProcessor, int targetTagNumber) {
        AprilTagDetection targetTag;
        boolean targetFound;
        double strafeError;
        double yawError;
        double strafePower = 1;
        double yawPower = 1;
        double minStrafePower = .01;
        double minYawPower = .01;
        while ((Math.abs(strafePower) > minStrafePower) || (Math.abs(yawPower) > minYawPower)) {
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
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData("Found", "ID %d (%s)", targetTag.id, targetTag.metadata.name);
                telemetry.addData("Range", "%5.1f inches", targetTag.ftcPose.range);
                telemetry.addData("Bearing", "%3.0f degrees", targetTag.ftcPose.bearing);
                telemetry.addData("Yaw", "%3.0f degrees", targetTag.ftcPose.yaw);

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                strafeError = targetTag.ftcPose.bearing;
                yawError = targetTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                strafePower = Range.clip(-yawError * Constants.atStrafeGain, -Constants.atMaxStrafe, Constants.atMaxStrafe);
                yawPower = Range.clip(strafeError * Constants.atYawGain, -Constants.atMaxYaw, Constants.atMaxYaw);

                telemetry.addData("Auto", "Strafe %5.2f", strafePower);
                telemetry.addData("Auto", "Yaw %5.2f", yawPower);
            }
        }
        telemetry.update();
        // Apply desired axes motions to the drivetrain.
        bot.moveDirection(0, strafePower, yawPower);
        sleep(10);
    }


    protected AprilTagProcessor createAprilTagProcessor() {
        // Create the AprilTag processor by using a builder.
        AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTagProcessor.setDecimation(2);

        return (aprilTagProcessor);
    }

    //   Manually set the camera gain and exposure.
    //   This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    protected VisionPortal createVisionPortal(int exposureMS, int gain, AprilTagProcessor
            aprilTagProcessor) {
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .build();

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }

        return (visionPortal);
    }
}