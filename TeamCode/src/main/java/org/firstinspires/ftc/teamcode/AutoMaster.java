package org.firstinspires.ftc.teamcode;

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

// Possible positions for the prop relative to the rigging
// Used to invert the prop search order for blue vs red
enum PropPosition {NEAR, MIDDLE, FAR}

// Starting position for the bot
enum StartPosition {FAR, NEAR}

enum Alliance {RED, BLUE}

// Target parking position
enum ParkPosition {CENTER, CORNER, NONE}

public abstract class AutoMaster extends LinearOpMode {
    protected Alliance alliance;
    protected StartPosition startPosition;
    protected ParkPosition parkPosition;
    protected Bot bot;

    protected AutoMaster(Alliance alliance, StartPosition startPosition, ParkPosition parkPosition) {
        this.alliance = alliance;
        this.startPosition = startPosition;
        this.parkPosition = parkPosition;
    }

    @Override
    public void runOpMode() {
        int riggingDirection;
        int boardDirection;
        int parkDirection;

        PropPosition propPosition;
        int targetAprilTagNumber;

        AprilTagProcessor aprilTagProcessor = null;
        VisionPortal visionPortal = null;
        bot = new Bot(this, Constants.maxAutoSpeed);

        riggingDirection = determineRiggingDirection();

        boardDirection = determineBoardDirection(riggingDirection);

        parkDirection = determineParkDirection(parkPosition, boardDirection);

        bot.wristDown();
        setToHighCruisingPosition();
        aprilTagProcessor = createAprilTagProcessor();

        visionPortal = createVisionPortal(Constants.atExposureMS, Constants.atExposureGain, aprilTagProcessor);

        sleep(1000);

        waitForStart();

        // Determine prop position, place the purple pixel on the spike mark, then go to escape position
        propPosition = dsPlacePurplePixel(riggingDirection);

        roughTravelToBoard(boardDirection, riggingDirection);

        targetAprilTagNumber = aprilTagNumber(propPosition, riggingDirection, boardDirection);
//        telemetry.addData("roughAlign: ",propPosition);
//        telemetry.update();
//        sleep(2500);
        roughAlignToAprilTag(boardDirection, targetAprilTagNumber, startPosition);
//        telemetry.addData("autoOrient: ",targetAprilTagNumber);
//        telemetry.update();
//        sleep(2500);

        // Correct strafe to directly face the target April Tag
        autoOrientToAprilTag(aprilTagProcessor, targetAprilTagNumber);

        bot.turnToHeading(boardDirection * -90);

        placePixelOnBoard();

        // Move straight until close to the wall, turn, and parallel park
        park(boardDirection, targetAprilTagNumber, parkDirection);

        // Lower lift, lower wrist, open grabber
        setStationaryPosition();
    }

    protected int determineRiggingDirection() {
        if (((startPosition == StartPosition.FAR) && (alliance == Alliance.BLUE)) ||
                ((startPosition == StartPosition.NEAR) && (alliance == Alliance.RED))) {
            return (-1);
        } else {
            return (1);
        }
    }

    protected int determineBoardDirection(int riggingDirection) {
        if (startPosition == StartPosition.FAR) {
            return (riggingDirection);
        } else {
            return (-riggingDirection);
        }
    }

    protected int determineParkDirection(ParkPosition parkPosition, int boardDirection) {
        if (parkPosition == ParkPosition.CORNER) {
            return (boardDirection);
        } else if (parkPosition == ParkPosition.CENTER) {
            return (-boardDirection);
        } else {
            return (0);
        }
    }

    protected void setToHighCruisingPosition() {
        bot.grabberClose();
        sleep(1000);
        bot.liftStopAtPosition(Constants.liftAutoHighCruisingPosition);
        sleep(1000);
        bot.wristUp();
    }

    protected void setToLowCruisingPosition() {
        bot.grabberClose();
        bot.wristUp();
        bot.liftStopAtPosition(Constants.liftAutoLowCruisingPosition);
    }

    protected PropPosition dsPlacePurplePixel(int riggingDirection) {
        double propDistance;
        PropPosition propPosition = null;
        double farHeading = riggingDirection * 90;
        double farSeekStrafe = 11.5 - Constants.sensorToDrivetrainMiddle;
        double farSeekMove = 17.5;
        double farPushMove = 7;
        double escapeStrafe = 11.5;
        double middleSeekMove = 8;
        double middlePushMove = 8;
        double nearPushMove = 5;

        if (riggingDirection > 0) {
            farSeekStrafe = farSeekStrafe + Constants.sensorToDrivetrainMiddle * 2;
        }

        // Scan for FAR position
        bot.moveStraightForDistance(farSeekMove);
        bot.strafeForDistance(-riggingDirection * farSeekStrafe);
        if (bot.getDistance() < Constants.dsPropDistanceThreshold) {
            propPosition = PropPosition.FAR;
            bot.strafeForDistance(Constants.sensorToDrivetrainMiddle);
            pushPixel(farPushMove);
            bot.strafeForDistance(-(riggingDirection * escapeStrafe));
        }

        // Scan for MIDDLE position
        if (propPosition == null) {
            bot.strafeForDistance(riggingDirection * farSeekStrafe);
            bot.moveStraightForDistance(middleSeekMove);
            bot.strafeForDistance(-Constants.sensorToDrivetrainMiddle);
            propDistance = bot.getDistance();
            bot.strafeForDistance(Constants.sensorToDrivetrainMiddle);
            if (propDistance < Constants.dsPropDistanceThreshold) {
                propPosition = PropPosition.MIDDLE;
                pushPixel(middlePushMove);
                bot.moveStraightForDistance(-middleSeekMove);
                bot.strafeForDistance(-(riggingDirection * (escapeStrafe + farSeekStrafe)));
            }
        }
        setToLowCruisingPosition();
        if (propPosition == null) {
            // If object not yet found, assumed NEAR position
            propPosition = PropPosition.NEAR;
            bot.turnToHeading(-farHeading);
            bot.strafeForDistance(-riggingDirection * Constants.sensorToDrivetrainMiddle);
            pushPixel(nearPushMove);
            bot.strafeForDistance(riggingDirection * Constants.sensorToDrivetrainMiddle);
            bot.turnToHeading(0);
            bot.moveStraightForDistance(-middleSeekMove);
            bot.strafeForDistance(-(riggingDirection * (escapeStrafe + farSeekStrafe)));
        }
        return (propPosition);
    }

    protected void pushPixel(double distance) {
        bot.moveStraightForDistance(distance + Constants.dsPlacementDistanceOffset);
        bot.moveStraightForDistance(-(distance + Constants.dsPlacementDistanceOffset));
    }

    protected void roughTravelToBoard(int boardPosition, int riggingDirection) {
        bot.turnToHeading(riggingDirection * 90);
    }

    protected int aprilTagNumber(PropPosition propPosition, int riggingDirection, int boardDirection) {
        int aprilTagNumber;

        if (riggingDirection == -1) {
            aprilTagNumber = 1;
            if (propPosition == PropPosition.MIDDLE) {
                aprilTagNumber = 2;
            } else if (propPosition == PropPosition.FAR) {
                aprilTagNumber = 3;
            }
        } else {
            aprilTagNumber = 1;
            if (propPosition == PropPosition.MIDDLE) {
                aprilTagNumber = 2;
            } else if (propPosition == PropPosition.NEAR) {
                aprilTagNumber = 3;
            }
        }

        if (boardDirection == 1) {
            aprilTagNumber = aprilTagNumber + 3;
        }
//        telemetry.addData("Tag #", aprilTagNumber);
//        telemetry.update();
//        sleep(2000);
        return (aprilTagNumber);
    }

    protected void roughAlignToAprilTag(int boardDirection,
                                        int targetAprilTagNumber, StartPosition startPosition) {
        double strafeVector = 0;
        double chassisWidth = 2 * Constants.sensorToDrivetrainMiddle;
        if (boardDirection == -1) {
            if (startPosition == StartPosition.FAR) {
                strafeVector = -((targetAprilTagNumber - 4 - 3) * Constants.distanceBetweenAprilTags);
            } else {
                strafeVector = chassisWidth + (targetAprilTagNumber - .6) * Constants.distanceBetweenAprilTags;
            }
        } else {
            if (startPosition == StartPosition.FAR) {
                strafeVector = chassisWidth + (targetAprilTagNumber - 4 + 3) * Constants.distanceBetweenAprilTags;
            } else {
                strafeVector = -(6 - targetAprilTagNumber) * Constants.distanceBetweenAprilTags;
            }
        }
        bot.strafeForDistance(strafeVector);
    }

    protected void autoOrientToAprilTag(AprilTagProcessor aprilTagProcessor, int targetTagNumber) {
//        telemetry.addData("Target: ",targetTagNumber);
//        telemetry.update();
//        sleep(500);

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

//            telemetry.addData("strafePower: ",strafePower);
//            telemetry.addData("yawPower: ",yawPower);
//            telemetry.update();
//            sleep(500);

            // Search through detected tags to find the target tag
            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
            for (AprilTagDetection detection : currentDetections) {
//                telemetry.addData("Iter Detections: ", targetFound);
//                telemetry.update();
//                sleep(100);

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


    protected void placePixelOnBoard() {
        bot.moveStraightForDistance(Constants.boardApproachDistance);
        bot.strafeForDistance(-Constants.sensorToDrivetrainMiddle);
        bot.liftStopAtPosition(Constants.liftAutoBoardPosition);
        bot.wristUp();
        sleep(500);
        bot.creepStraightForDistance(bot.getDistance() - Constants.boardOffsetDistance);
//        bot.wristMiddle();
//        bot.creepUntilContact();
//        bot.moveStraightForDistance(-2);
//        bot.moveStraightForDistance(2);
//        sleep(500);
        bot.grabberOpen();
        sleep(250);
        bot.liftStopAtPosition(Constants.liftAutoBoardPosition + 150);
        sleep(250);
        bot.moveStraightForDistance(-Constants.boardEscapeDistance);
    }

    protected void park(double boardDirection, int targetAprilTagNumber, double parkDirection) {
        double strafeVector = 0;
        int adjustedTagNumber = targetAprilTagNumber;
        if (targetAprilTagNumber > 3) {
            adjustedTagNumber = adjustedTagNumber - 3;
        }
        strafeVector = 2.5 * Constants.sensorToDrivetrainMiddle;

        if (parkDirection > 0) {
            strafeVector = strafeVector + (4.5 - adjustedTagNumber) * Constants.distanceBetweenAprilTags;
        } else {
            strafeVector = -strafeVector - adjustedTagNumber * Constants.distanceBetweenAprilTags;
        }

        bot.turnToHeading(boardDirection * 90);
        bot.strafeForDistance(-strafeVector);
        bot.moveStraightForDistance(-14);
        setStationaryPosition();
        sleep(2500);
    }

    protected void setStationaryPosition() {
        bot.grabberClose();
        sleep(100);
        bot.wristDown();
        bot.liftStopAtPosition(0);
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