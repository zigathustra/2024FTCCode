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
//        telemetry.addData("escaped", direction);
//        telemetry.update();
//        sleep(3000);

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
        double objectDistance = 0;
        double strafeDistance = 0;
        Position objectPosition = Position.NEAR; // Default position
        double straightDistance = 0;

        double middleHeading = Constants.dsMiddlePositionHeading;
        double nearHeading = Constants.dsRightPositionHeading;
        double farHeading = Constants.dsLeftPositionHeading;

        double middleStrafeDistance = Constants.dsMiddlePositionStrafeDistance;
        double nearStrafeDistance = Constants.dsRightPositionStrafeDistance;
        double farStrafeDistance1 = Constants.dsLeftPositionStrafeDistance;
        double farStrafeDistance2 = 0;
        double straightDistance1 = 10;
        double straightDistance2 = 8;
        double defaultObjectDistance = 5;

        if (alliance == 2) {
            middleHeading = Constants.dsMiddlePositionHeading;
            nearStrafeDistance = Constants.dsLeftPositionStrafeDistance;
            farStrafeDistance1 = Constants.dsRightPositionStrafeDistance;
            farStrafeDistance2 = 6;
        }

        strafeDistance = middleStrafeDistance;
        bot.strafeForDistance(-strafeDistance);
        objectDistance = bot.getDistance();
        if (objectDistance < Constants.dsPropDistanceThreshold) {
            objectPosition = Position.MIDDLE;
        } else {
            bot.strafeForDistance(strafeDistance);
            bot.turnToHeading(allianceDirection*nearHeading);
            strafeDistance = allianceDirection*nearStrafeDistance;
            bot.strafeForDistance(-strafeDistance);
            objectDistance = bot.getDistance();
            if (objectDistance < Constants.dsPropDistanceThreshold) {
                objectPosition = Position.NEAR;
            } else {
                objectPosition = Position.FAR;
                bot.moveStraightForDistance(straightDistance1);
                bot.strafeForDistance(strafeDistance);
                bot.turnToHeading(allianceDirection*farHeading);
                strafeDistance = farStrafeDistance1;
                bot.strafeForDistance(-strafeDistance);
                straightDistance = straightDistance2;
                bot.moveStraightForDistance(straightDistance);
            }
        }
        objectDistance = bot.getDistance();
        if ((objectDistance > 14) || (objectDistance < 2)) {
            objectDistance = defaultObjectDistance;
        }
        bot.strafeForDistance(farStrafeDistance2);
        bot.moveStraightForDistance(Constants.dsPlacementDistanceOffset + objectDistance);
        bot.moveStraightForDistance(-Constants.dsPlacementDistanceOffset - objectDistance);
        bot.moveStraightForDistance(-straightDistance);
        bot.strafeForDistance(strafeDistance-farStrafeDistance2);
        propPosition = objectPosition;
    }

    protected void escapeSquare() {
        double moveDistance = 28;
        double strafeDistance = 25;
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
            bot.moveStraightForDistance(moveDistance - 16);
            bot.strafeForDistance(-(allianceDirection * strafeDistance));
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
//        bot.moveStraightToObject(15);
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