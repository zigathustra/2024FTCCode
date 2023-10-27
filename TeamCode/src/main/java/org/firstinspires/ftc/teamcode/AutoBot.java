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
        super(opMode);
 //       opMode.telemetry.addData("Status: ", "AutoBot lives!");
 //       opMode.telemetry.update();
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
        final double axialGain = 0.02;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
        final double strafeGain = 0.01;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
        final double yawGain = 0.01;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
        final double maxAxial = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
        final double maxStrafe = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
        final double maxYaw = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
        double scanAngle = 2;
        final double maxTotalAngle = 15;
        final double minTotalAngle = -15;
        double totalAngle = 0;

        initAprilTag();

        setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        while (axialError > axialErrorThreshold) {
            targetFound = false;
            targetTag = null;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
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
                opMode.telemetry.addData("\n>", "HOLD Left-Bumper to Drive to Target\n");
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
                axialPower = Range.clip(axialError * axialGain, -maxAxial, maxAxial);
                yawPower = Range.clip(strafeError * yawGain, -maxYaw, maxYaw);
                strafePower = Range.clip(-yawError * strafeGain, -maxStrafe, maxStrafe);

                opMode.telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", axialPower, strafePower, yawPower);
            } else {
                // scan for tags
                if ((totalAngle >= maxTotalAngle) || (totalAngle <= minTotalAngle)){
                    scanAngle = -scanAngle;
                }
                totalAngle = totalAngle + scanAngle;
                turnForDistance(scanAngle);
                axialPower = 0.0;
                strafePower = 0.0;
                yawPower = 0.0;
                opMode.telemetry.addData("Manual", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", axialPower, strafePower, yawPower);
            }
            opMode.telemetry.update();

            // Apply desired axes motions to the drivetrain.
            driveTrain.moveDirection(axialPower, strafePower, yawPower);
            opMode.sleep(10);

        }
    }

    // Scan for prop in one of three positions
    // Return position 1, 2, or 3 (left, right, or center)
    public int dsPlacePurplePixel(){
        final double propDistanceThreshold = 10;
        final int position1Heading = -270;
        final int position2Heading = 0;
        final int position3Heading = -90;
        final double placementDistanceOffset = 5.5;
        double objectDistance = 0;

        final double position1StrafeDistance = 3;
        final double position2StrafeDistance = 3.5;
        final double position3StrafeDistance = 12;
        double strafeDistance = 0;

        int objectPosition = 3;
        boolean objectFound = false;

//        opMode.telemetry.addData("dsDetObjPos", "");
//        opMode.telemetry.update();
//        opMode.sleep(3000);
        strafeDistance = position2StrafeDistance;
        turnToHeading(position2Heading);
        strafeForDistance(-strafeDistance);
        objectDistance = distanceSensor.getDistance(DistanceUnit.INCH);
        if (objectDistance < propDistanceThreshold) {
            objectFound = true;
            objectPosition = 2;
        }
  //      opMode.telemetry.addData("objectFound", objectFound);
  //      opMode.telemetry.update();
  //      opMode.sleep(3000);
        if (!objectFound){
            strafeForDistance(strafeDistance);
//            opMode.telemetry.addData("heading", object3Heading);
//            opMode.telemetry.update();
//            opMode.sleep(3000);
//            opMode.sleep(500);
            strafeDistance = position3StrafeDistance;
            turnToHeading(position3Heading);
            strafeForDistance(-strafeDistance);
            objectDistance = distanceSensor.getDistance(DistanceUnit.INCH);
            if (objectDistance < propDistanceThreshold) {
                objectFound = true;
                objectPosition = 3;
            } else {
                objectFound = true;
                objectPosition = 1;
                strafeForDistance(strafeDistance);
//                wristDown();
                turnToHeading(position1Heading);
//                wristUp();
                strafeDistance = -position1StrafeDistance;
                strafeForDistance(-strafeDistance);
            }
//            opMode.telemetry.addData("objectPosition", objectPosition);
//            opMode.telemetry.update();
//            opMode.sleep(3000);
        }
        objectDistance = distanceSensor.getDistance(DistanceUnit.INCH);
        moveStraightForDistance(placementDistanceOffset + objectDistance);
        moveStraightForDistance(-placementDistanceOffset - objectDistance);
        strafeForDistance(strafeDistance);
//        opMode.telemetry.addData("objectPosition", objectPosition);
//        opMode.telemetry.update();
//        opMode.sleep(3000);
        return(objectPosition);
    }

    public void moveStraightToObject(double targetDistance) {
        double objectDistance = distanceSensor.getDistance(DistanceUnit.INCH);
        moveStraightForDistance(objectDistance - targetDistance);
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

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
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


