package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sun.tools.javac.util.Abort;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public abstract class AutoMaster extends LinearOpMode {
    protected AutoBot bot = null;
    protected int alliance = 0;   // 1=Red, 2=Blue
    private int propPosition = 0;
    private int targetAprilTagNumber = 0;
    protected int direction = 0;

    public AutoMaster(int alliance) {
        this.alliance = alliance;
    }

    @Override
    public void runOpMode() {
        bot = new AutoBot(this);
        if (alliance == 2) {
            direction = -1;
        } else {
            direction = 1;
        }
        waitForStart();

        // Raise lift, raise wrist, close grabber
        setToCruisingPosition();

        // Move to the center of the spike mark square
        moveToCenterOfSquare();

        // Determine prop position, and place the purple pixel on the spike mark
        dsPlacePurplePixel();
        targetAprilTagNumber = propPosition;
        if (alliance == 1) {
            targetAprilTagNumber = targetAprilTagNumber + 3;
        }
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
        double strafeVector = 0;
        int objectPosition = 3; // Default position

        strafeVector = direction * Constants.dsPosition2StrafeDistance;
        bot.strafeForDistance(-strafeVector);
        objectDistance = bot.getDistance();
        if (objectDistance < Constants.dsPropDistanceThreshold) {
            objectPosition = 2;
        } else {
            bot.strafeForDistance(strafeVector);
            strafeVector = direction * Constants.dsPosition3StrafeDistance;
            telemetry.addData("direction: ", direction);
            telemetry.addData("pos 3 heading", Constants.dsPosition3Heading);
            telemetry.addData("alliance: ", alliance);
            telemetry.update();
            sleep(3000);
            bot.turnToHeading(direction * Constants.dsPosition3Heading);
            bot.strafeForDistance(-strafeVector);
            objectDistance = bot.getDistance();
            if (objectDistance < Constants.dsPropDistanceThreshold) {
                objectPosition = 3;
            } else {
                objectPosition = 1;
                bot.moveStraightForDistance(10);
                bot.strafeForDistance(strafeVector);
                bot.turnToHeading(direction * Constants.dsPosition1Heading);
                strafeVector = -(direction * Constants.dsPosition1StrafeDistance);
                bot.strafeForDistance(-strafeVector);
                bot.moveStraightForDistance(8);
            }
        }
        objectDistance = bot.getDistance();
        if ((objectDistance > 14) || (objectDistance < 2)) {
            objectDistance = 8;
        }
        bot.moveStraightForDistance(Constants.dsPlacementDistanceOffset + objectDistance);
        bot.moveStraightForDistance(-Constants.dsPlacementDistanceOffset - objectDistance);
        bot.strafeForDistance(strafeVector);
        propPosition = objectPosition;
    }

    protected void escapeSquare() {
        double moveDistance = 28;
        double strafeDistance = 25;
        double heading = direction * Constants.dsPosition3Heading;
        if (propPosition == 1) {
            bot.moveStraightForDistance(-12);
            bot.turnToHeading(heading);
            bot.moveStraightForDistance(moveDistance - 16);
            bot.strafeForDistance(-(direction * (strafeDistance - 3)));
        } else {
            if (propPosition == 3) {
                bot.strafeForDistance(-(direction * strafeDistance));
                bot.moveStraightForDistance(moveDistance);
            } else {
                bot.turnToHeading(heading);
                bot.moveStraightForDistance(moveDistance);
                bot.strafeForDistance(-(direction * strafeDistance));
            }
        }
    }

    protected void moveToCenterOfSquare() {
        bot.moveStraightForDistance(25);
    }

    protected void moveStraightAndPark() {
//        bot.moveStraightToObject(15);
        double strafeVector = direction * 12;
        bot.moveStraightForDistance(9);
        bot.turnToHeading(0);
        bot.strafeForDistance(strafeVector);
        bot.stopDrive();
    }

    protected void setStationaryPosition() {
        bot.wristDown();
        bot.grabberOpen();
        bot.liftStopAtPosition(0);
    }

}
