package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sun.tools.javac.util.Abort;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

enum Position {NEAR, MIDDLE, FAR};

public abstract class AutoMaster extends LinearOpMode {
    protected AutoBot bot = null;
    protected int alliance = 0;   // 1=Red, 2=Blue
    protected Position propPosition = Position.NEAR;
    protected int targetAprilTagNumber = 0;
    protected int allianceDirection = 0;

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
//        targetAprilTagNumber = propPosition;
//        if (alliance == 1) {
//            targetAprilTagNumber = targetAprilTagNumber + 3;
//        }
        //       telemetry.addData("placed", propPosition);
        //       telemetry.update();
        //       sleep(3000);

        // Leave the spike mark square and move to a square that faces the parking position
        escapeSquare();
//        telemetry.addData("escaped", allianceDirection);
//        telemetry.update();
//        sleep(3000);

        // Move straight until close to the wall, turn, and parallel park
        moveStraightAndPark();
//        telemetry.addData("parked", allianceDirection);
//        telemetry.update();
//        sleep(3000);
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
        double defaultObjectDistance = 5;

        if (alliance == 2) {
            nearStrafeDistance = Constants.dsLeftPositionStrafeDistance + 1.5;
            farStrafeDistance1 = Constants.dsRightPositionStrafeDistance;
            farStrafeDistance2 = 6;
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
//            telemetry.addData("sd1", farStrafeDistance1);
//            telemetry.addData("sd2", farStrafeDistance2);
//            telemetry.addData("od", objectDistance);
//            telemetry.update();
//                    sleep(5000);
            placePixel(objectDistance);
            bot.moveStraightForDistance(-farDistance2);
//            bot.strafeForDistance(-(allianceDirection * farStrafeDistance1));
//            bot.strafeForDistance(-farStrafeDistance2);
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
        double strafeDistance = 27;
        double farStraightDistance = 4;
        double boardHeading = allianceDirection * Constants.dsRightPositionHeading;

//        telemetry.addData("allianceDirection: ", allianceDirection);
//        telemetry.addData("boardHeading: ", boardHeading);
//        telemetry.addData("propPosition: ", propPosition);
//        telemetry.update();
//        sleep(5000);

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
        double strafeDistance = allianceDirection * 12;
        bot.moveStraightForDistance(9);
        bot.turnToHeading(0);
        bot.strafeForDistance(strafeDistance);
        bot.stopDrive();
    }

    protected void setStationaryPosition() {
        bot.wristDown();
        bot.grabberOpen();
        bot.liftStopAtPosition(0);
    }

}