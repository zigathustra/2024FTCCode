package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
enum Position {NEAR, MIDDLE, FAR};

public abstract class AutoMaster extends LinearOpMode {
    protected AutoBot bot = null;
    protected int alliance = 0;   // 1=Red, 2=Blue
    protected Position propPosition = Position.NEAR;
    protected int targetAprilTagNumber = 0;
    protected int allianceDirection = 0;
    private AprilTagProcessor aprilTagProcessor = null;
    private VisionPortal visionPortal = null;

    public AutoMaster(int alliance) {
        this.alliance = alliance;
    }

    @Override
    public void runOpMode() {
        bot = new AutoBot(this);
        if (alliance == 2) {
            allianceDirection = -1;
        } else {
            allianceDirection = 1;
        }

        waitForStart();

        // Raise lift, raise wrist, close grabber
        setToCruisingPosition();

        // Move to the center of the spike mark square
        moveToCenterOfSquare();

        // Determine prop position, and place the purple pixel on the spike mark
        dsPlacePurplePixel();

        if (alliance == 1)
        {
            if (propPosition == Position.NEAR){targetAprilTagNumber = 1;}

            if (propPosition == Position.MIDDLE){targetAprilTagNumber = 2;}

            if (propPosition == Position.FAR){targetAprilTagNumber = 3;}
        }
        else if (alliance == 2)
        {
            if (propPosition == Position.FAR){targetAprilTagNumber = 4;}

            if (propPosition == Position.MIDDLE){targetAprilTagNumber = 5;}

            if (propPosition == Position.NEAR){targetAprilTagNumber = 6;}
        }

        // Leave the spike mark square and move to a square that faces the parking position
        escapeSquare();

        // Move straight until close to the wall, turn, and parallel park
        moveStraightAndPark();

        // Lower lift, lower wrist, open grabber
        setStationaryPosition();
    }

    protected void setToCruisingPosition() {
        bot.wristUp();
        bot.grabberClose();
        bot.liftStopAtPosition(550);
    }

    // Scan for prop in one of three positions
    // Return position 1, 2, or 3 (far from board, middle, close to board)
    protected void dsPlacePurplePixel() {
        boolean objectFound = false;
        double objectDistance = 0;
        double strafeDistance = 0;
        Position objectPosition = Position.NEAR; // Default position
        double straightDistance = 0;

        double nearHeading = Constants.dsRightPositionHeading;
        double farHeading = Constants.dsLeftPositionHeading;

        double middleStrafeDistance = Constants.dsMiddlePositionStrafeDistance;
        double nearStrafeDistance = Constants.dsRightPositionStrafeDistance;
        double farStrafeDistance1 = Constants.dsLeftPositionStrafeDistance;
        double farStrafeDistance2 = 0;
        double farDistance1 = 10;
        double farDistance2 = 8;
        double defaultObjectDistance = 3;

        if (alliance == 2) {
            nearStrafeDistance = Constants.dsLeftPositionStrafeDistance + 1.5;
            farStrafeDistance1 = Constants.dsRightPositionStrafeDistance;
            farStrafeDistance2 = 10;
        }

        // Scan for middle position
        bot.strafeForDistance(-middleStrafeDistance);
        objectDistance = bot.getDistance();
        if (objectDistance < Constants.dsPropDistanceThreshold) {
            objectFound = true;
            objectPosition = Position.MIDDLE;
            placePixel(objectDistance);
        }
        bot.strafeForDistance(middleStrafeDistance);

        // Scan for near position
        if (!objectFound) {
            bot.turnToHeading(allianceDirection * nearHeading);
            bot.strafeForDistance(-(allianceDirection*nearStrafeDistance));
            objectDistance = bot.getDistance();
            if (objectDistance < Constants.dsPropDistanceThreshold) {
                objectFound = true;
                objectPosition = Position.NEAR;
                placePixel(objectDistance);
            }
            bot.strafeForDistance(allianceDirection*nearStrafeDistance);
        }

        // If object not yet found, assumed far position
        if (!objectFound) {
            objectFound = true;
            objectPosition = Position.FAR;
            bot.moveStraightForDistance(farDistance1);
            bot.turnToHeading(allianceDirection * farHeading);
            bot.strafeForDistance(allianceDirection * farStrafeDistance1);
            bot.moveStraightForDistance(farDistance2);
            objectDistance = validateDistance(2, 14, bot.getDistance(), defaultObjectDistance);
            bot.strafeForDistance(farStrafeDistance2);
            placePixel(objectDistance);
            bot.moveStraightForDistance(-farDistance2);
        }
        propPosition = objectPosition;
    }

    protected double validateDistance(double min, double max, double objectDistance, double defaultDistance) {
        if ((objectDistance > max) || (objectDistance < min)) {
            return (defaultDistance);
        }
        return (objectDistance);
    }

    protected void placePixel(double objectDistance) {
        bot.moveStraightForDistance(Constants.dsPlacementDistanceOffset + objectDistance);
        bot.moveStraightForDistance(-Constants.dsPlacementDistanceOffset - objectDistance);
    }

    protected void escapeSquare() {
        double moveDistance = 26;
        double strafeDistance = 17;
        double farStraightDistance = 4;
        double boardHeading = allianceDirection * Constants.dsRightPositionHeading;

        if (propPosition == Position.FAR) {
            bot.moveStraightForDistance(-farStraightDistance);
            bot.turnToHeading(boardHeading);
            bot.moveStraightForDistance(moveDistance - 12);
            bot.strafeForDistance(-(allianceDirection * (strafeDistance - 8)));
        } else {
            if (propPosition == Position.NEAR) {
                bot.strafeForDistance(-(allianceDirection * strafeDistance));
                bot.moveStraightForDistance(moveDistance);
            } else {
                bot.turnToHeading(boardHeading);
                bot.moveStraightForDistance(moveDistance);
                bot.strafeForDistance(-(allianceDirection * strafeDistance));
            }
        }
    }

    protected void moveToCenterOfSquare() {
        bot.moveStraightForDistance(25);
    }

    protected void moveStraightAndPark() {
        double strafeDistance = allianceDirection * 4;
                bot.liftStopAtPosition(0);
        bot.moveStraightForDistance(8);
        bot.turnToHeading(0);
        bot.strafeForDistance(strafeDistance);
        bot.stopDrive();
    }

    protected void setStationaryPosition() {
        bot.wristDown();
        bot.liftStopAtPosition(0);
    }


    public void autoOrientToAprilTag(int targetTagNumber) {
        AprilTagDetection targetTag = null;
        boolean targetFound = false;
        double axialError = 1000;
        double strafeError = 1000;
        double yawError = 1000;
        double axialPower = 1;
        double strafePower = 1;
        double yawPower = 1;
        double minStrafePower = .001;
        double minYawPower = 0.01;

 
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
                axialError = (targetTag.ftcPose.range);
                strafeError = targetTag.ftcPose.bearing;
                yawError = targetTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                axialPower = Range.clip(axialError * Constants.atAxialGain, -Constants.atMaxAxial, Constants.atMaxAxial);
                yawPower = Range.clip(strafeError * Constants.atYawGain, -Constants.atMaxYaw, Constants.atMaxYaw);
                strafePower = Range.clip(-yawError * Constants.atStrafeGain, -Constants.atMaxStrafe, Constants.atMaxStrafe);

                telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", axialPower, strafePower, yawPower);
            } 
            telemetry.update();
            // Apply desired axes motions to the drivetrain.
            bot.moveDirection(0, strafePower, yawPower);
            sleep(10);
        }
    }



    
    public void autoDriveToAprilTag(int targetTagNumber, double targetDistanceFromTag) {
        AprilTagDetection targetTag = null;
        boolean targetFound = false;
        double axialError = 1000;
        double strafeError = 1000;
        double yawError = 1000;
        double minAxialPower = 0.01;
        double minStrafePower = 0.01;
        double minYawPower = 0.01;
        double axialPower = 1;
        double strafePower = 1;
        double yawPower = 1;


        while ((Math.abs(axialPower) > minAxialPower) || (Math.abs(strafePower) > minStrafePower) || (Math.abs(yawPower) > minYawPower)) {
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
                axialError = (targetTag.ftcPose.range - targetDistanceFromTag);
                strafeError = targetTag.ftcPose.bearing;
                yawError = targetTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                axialPower = Range.clip(axialError * Constants.atAxialGain, -Constants.atMaxAxial, Constants.atMaxAxial);
                yawPower = Range.clip(strafeError * Constants.atYawGain, -Constants.atMaxYaw, Constants.atMaxYaw);
                strafePower = Range.clip(-yawError * Constants.atStrafeGain, -Constants.atMaxStrafe, Constants.atMaxStrafe);

                telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", axialPower, strafePower, yawPower);
            } 
            telemetry.update();
            // Apply desired axes motions to the drivetrain.
            bot.moveDirection(axialPower, strafePower, yawPower);
            sleep(10);
        }
    }
        protected void initAprilTag() {
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
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .build();
    }
    
    

    //   Manually set the camera gain and exposure.
    //   This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    protected void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls
        if (visionPortal == null) {
            return;
        }

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
    }
}