package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "AutoCloseBlue528", group = "Autonomous")
public class AutoCloseBlue528 extends AutoMaster {
    public AutoCloseBlue528() {
        super(2);
    }
/*
    // Scan for prop in one of three positions
    // Return position 1, 2, or 3 (left, right, middle)
    protected void dsPlacePurplePixel() {
        double objectDistance = 0;
        double strafeVector = 0;
        int objectPosition = 1; // Default position

        strafeVector = Constants.dsMiddlePositionStrafeDistance;
        bot.strafeForDistance(-strafeVector);
        objectDistance = bot.getDistance();
        if (objectDistance < Constants.dsPropDistanceThreshold) {
            objectPosition = 2;
        } else {
            bot.strafeForDistance(strafeVector);
            strafeVector = direction * Constants.dsLeftPositionStrafeDistance;
            bot.turnToHeading(Constants.dsLeftPositionHeading);
            bot.strafeForDistance(-strafeVector);
            objectDistance = bot.getDistance();
            if (objectDistance < Constants.dsPropDistanceThreshold) {
                objectPosition = 1;
            } else {
                objectPosition = 3;
                bot.moveStraightForDistance(10);
                bot.strafeForDistance(strafeVector);
                bot.turnToHeading(Constants.dsRightPositionHeading);
//                strafeVector = -(direction * Constants.dsRightPositionStrafeDistance);
                bot.strafeForDistance(-5.5);
                bot.moveStraightForDistance(8);
            }
        }
        objectDistance = bot.getDistance();
        if ((objectDistance > 14) || (objectDistance < 2)) {
            objectDistance = 8;
        }
        objectDistance = 6;
        bot.moveStraightForDistance(Constants.dsPlacementDistanceOffset + objectDistance);
        bot.moveStraightForDistance(-Constants.dsPlacementDistanceOffset - objectDistance);
        bot.strafeForDistance(strafeVector);
        propPosition = objectPosition;    }
    protected void escapeSquare() {
        double moveDistance = 28;
        double strafeDistance = 25;
        double heading = direction * Constants.dsRightPositionHeading;
        if (propPosition == 3) {
            bot.moveStraightForDistance(-12);
            bot.turnToHeading(heading);
            bot.moveStraightForDistance(moveDistance - 16);
            bot.strafeForDistance(-(direction * (strafeDistance - 3)));
        } else {
            if (propPosition == 1) {
                bot.strafeForDistance(-(direction * strafeDistance));
                bot.moveStraightForDistance(moveDistance);
            } else {
                bot.turnToHeading(heading);
                bot.moveStraightForDistance(moveDistance);
                bot.strafeForDistance(-(direction * strafeDistance));
            }
        }
    }

 */
}
