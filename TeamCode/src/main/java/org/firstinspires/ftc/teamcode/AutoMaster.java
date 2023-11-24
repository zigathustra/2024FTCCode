package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

// Possible positions for the prop relative to the rigging
// Used to invert the prop search order for blue vs red
enum PropPosition {NEAR, MIDDLE, FAR}

// Starting position for the bot
enum BotPosition {RED_FAR, RED_NEAR, BLUE_FAR, BLUE_NEAR}

// Target parking position
//enum ParkPosition {CENTER, CORNER}

public abstract class AutoMaster extends LinearOpMode {
    protected Bot bot = null;
    protected final BotPosition botStartPosition;
    protected final double boardHeading;
    private AprilTagProcessor aprilTagProcessor = null;
    private VisionPortal visionPortal = null;

    public AutoMaster(BotPosition botStartPosition) {
        this.botStartPosition = botStartPosition;
//        this.parkPosition = parkPosition;
        boardHeading = boardDirectionFactor() * -90;
    }

    @Override
    public void runOpMode() {
        PropPosition propPosition;
        int targetAprilTagNumber;
        createBot();

//        bot.grabberClose();

        initAprilTag();

        setManualExposure(Constants.atExposureMS, Constants.atExposureGain);  // Use low exposure time to reduce motion blur

        waitForStart();

        // Raise lift, raise wrist, close grabber
        //setToCruisingPosition();

        //sleep(300);

        // Determine prop position, place the purple pixel on the spike mark, then go to escape position
        propPosition = dsPlacePurplePixel();

        targetAprilTagNumber = aprilTagNumber(propPosition);

        // Correct strafe and yaw to directly face the target April Tag
        autoOrientToAprilTag(targetAprilTagNumber);

        bot.turnToHeading(boardHeading);

        placePixelOnBoard();

//        // Move straight until close to the wall, turn, and parallel park
        park(parkStrafeVector(targetAprilTagNumber));
//
//        // Lower lift, lower wrist, open grabber
//        setStationaryPosition();
    }

    protected void createBot() {
        bot = new Bot(this, Constants.maxAutoSpeed);
    }

    protected abstract double boardDirectionFactor(); // 1 = right, -1 = left

    protected abstract double riggingDirectionFactor(); // 1 = right, -1 = left

    protected abstract double parkStrafeVector(int targetAprilTagNumber);

    protected void setToCruisingPosition() {
//        bot.wristUp();
//        bot.grabberClose();
//        bot.liftStopAtPosition(550);
    }

    protected PropPosition dsPlacePurplePixel() {
        double propDistance;
        PropPosition propPosition = null;
        double farSeekStrafe = 11.5 - Constants.sensorToDrivetrainMiddle;
        double farSeekMove = 1.25 * Constants.drivetrainLength;
        double farPushMove = 10;
        double escapeMove = 0.90 * Constants.drivetrainLength;
        double middleSeekMove = 8;
        double middlePushMove = 12;
        double nearPushMove = 12;
        double boardCorrectionFactor = 0;

        if (riggingDirectionFactor() < 1)
        {
            farSeekStrafe = farSeekStrafe + Constants.sensorToDrivetrainMiddle * 2;
        }

        boardCorrectionFactor = -boardDirectionFactor();

        // Scan for FAR position
        bot.strafeForDistance(-riggingDirectionFactor() * farSeekStrafe);
        bot.moveStraightForDistance(farSeekMove);
        if (bot.getDistance() < Constants.dsPropDistanceThreshold) {
            propPosition = PropPosition.FAR;
            bot.strafeForDistance(Constants.sensorToDrivetrainMiddle);
            pushPixel(farPushMove);
            bot.turnToHeading(boardHeading);
            bot.moveStraightForDistance(escapeMove);
//            telemetry.addData("Strafe: ", (boardDirectionFactor() * -(boardCorrectionFactor * 2 * Constants.sensorToDrivetrainMiddle)));
//            telemetry.update();
//            sleep(3000);

            bot.strafeForDistance(boardDirectionFactor() * -(boardCorrectionFactor * 2 * Constants.sensorToDrivetrainMiddle));
        }

        // Scan for MIDDLE position
        if (propPosition == null) {
            bot.strafeForDistance(-riggingDirectionFactor() * farSeekStrafe);
            bot.moveStraightForDistance(middleSeekMove);
            bot.strafeForDistance(-Constants.sensorToDrivetrainMiddle);
            propDistance = bot.getDistance();
            bot.strafeForDistance(Constants.sensorToDrivetrainMiddle);
            if (propDistance < Constants.dsPropDistanceThreshold) {
                propPosition = PropPosition.MIDDLE;
                pushPixel(middlePushMove);
                bot.moveStraightForDistance(-middleSeekMove);
                bot.turnToHeading(boardHeading);
                bot.moveStraightForDistance(escapeMove + farSeekStrafe + Constants.sensorToDrivetrainMiddle);
                bot.strafeForDistance(boardDirectionFactor() * -(Constants.distanceBetweenAprilTags + boardCorrectionFactor * 2 * Constants.sensorToDrivetrainMiddle));
            }
        }

        if (propPosition == null) {
            // If object not yet found, assumed NEAR position
            propPosition = PropPosition.NEAR;
            bot.turnToHeading(-boardHeading);
            bot.strafeForDistance(riggingDirectionFactor() * Constants.sensorToDrivetrainMiddle);
            pushPixel(nearPushMove);
            bot.strafeForDistance(-riggingDirectionFactor() * Constants.sensorToDrivetrainMiddle);
            bot.turnToHeading(0);
            bot.moveStraightForDistance(-middleSeekMove);
            bot.turnToHeading(boardHeading);
            bot.moveStraightForDistance(escapeMove + farSeekStrafe + Constants.sensorToDrivetrainMiddle);
            bot.strafeForDistance(boardDirectionFactor() * -(2 * Constants.distanceBetweenAprilTags + boardCorrectionFactor * 2 * Constants.sensorToDrivetrainMiddle));
        }
        return (propPosition);
    }

    protected void pushPixel(double distance) {
        bot.moveStraightForDistance(distance);
        bot.moveStraightForDistance(-distance);
    }

    protected void placePixelOnBoard() {
        double boardDistance;
        bot.moveStraightForDistance(Constants.boardApproachDistance);
        bot.strafeForDistance(-Constants.sensorToDrivetrainMiddle);
//        bot.liftStopAtPosition(750);
        //creepToContact();
        boardDistance = bot.getDistance();
        bot.creepStraightForDistance(boardDistance - Constants.boardOffsetDistance);
//        bot.grabberOpen();
        bot.moveStraightForDistance(-Constants.boardEscapeDistance);
    }

    protected void park(double parkStrafeVector) {
//        bot.liftStopAtPosition(0);
//        telemetry.addData("vector",parkStrafeVector);
//        telemetry.update();
//        sleep(5000);
        bot.strafeForDistance(parkStrafeVector);
        bot.turnToHeading(-boardHeading);
        bot.moveStraightForDistance(-15);
        bot.stopDrive();
    }

    protected void setStationaryPosition() {
//        bot.wristDown();
//        bot.liftStopAtPosition(0);
    }

    protected abstract int aprilTagNumber(PropPosition propPosition);

    public void autoStrafeToAprilTag(int targetTagNumber) {
        AprilTagDetection targetTag;
        boolean targetFound;
        double strafeError;
        double yawError;
        double strafePower = 1;
//        double yawPower = 1;
        double minStrafePower = .01;
        double minYawPower = 0.01;

        while (Math.abs(strafePower) > minStrafePower) {
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
//                yawPower = Range.clip(strafeError * Constants.atYawGain, -Constants.atMaxYaw, Constants.atMaxYaw);
                strafePower = Range.clip(-yawError * Constants.atStrafeGain, -Constants.atMaxStrafe, Constants.atMaxStrafe);

                telemetry.addData("Auto", "Strafe %5.2f", strafePower);
            }
            telemetry.update();
            // Apply desired axes motions to the drivetrain.
            bot.moveDirection(0, strafePower, 0);
            sleep(10);
        }
    }

    public void autoOrientToAprilTag(int targetTagNumber) {
        AprilTagDetection targetTag;
        boolean targetFound;
        double strafeError;
        double yawError;
        double strafePower = 1;
        double yawPower = 1;
        double minStrafePower = .01;
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
                strafeError = targetTag.ftcPose.bearing;
                yawError = targetTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                yawPower = Range.clip(strafeError * Constants.atYawGain, -Constants.atMaxYaw, Constants.atMaxYaw);
                strafePower = Range.clip(-yawError * Constants.atStrafeGain, -Constants.atMaxStrafe, Constants.atMaxStrafe);

                telemetry.addData("Auto", "Strafe %5.2f, Turn %5.2f ", strafePower, yawPower);
            }
            telemetry.update();
            // Apply desired axes motions to the drivetrain.
            bot.moveDirection(0, strafePower, yawPower);
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