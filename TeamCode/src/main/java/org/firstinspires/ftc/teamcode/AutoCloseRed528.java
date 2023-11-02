package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "AutoCloseRed528", group = "Autonomous")
public class AutoCloseRed528 extends AutoMaster {
    public AutoCloseRed528()
    {
        super(1);
    }
 /*   protected void dsPlacePurplePixel() {
        double objectDistance = 0;
        double strafeVector = 0;
        int objectPosition = 3; // Default position

        strafeVector = direction * Constants.dsMiddlePositionStrafeDistance;
        bot.strafeForDistance(-strafeVector);
        objectDistance = bot.getDistance();
        if (objectDistance < Constants.dsPropDistanceThreshold) {
            objectPosition = 2;
        } else {
            bot.strafeForDistance(strafeVector);
            strafeVector = direction * Constants.dsRightPositionStrafeDistance;
            bot.turnToHeading(direction * Constants.dsRightPositionHeading);
            bot.strafeForDistance(-strafeVector);
            objectDistance = bot.getDistance();
            if (objectDistance < Constants.dsPropDistanceThreshold) {
                objectPosition = 3;
            } else {
                objectPosition = 1;
                bot.moveStraightForDistance(10);
                bot.strafeForDistance(strafeVector);
                bot.turnToHeading(direction * Constants.dsLeftPositionHeading);
                strafeVector = -(direction * Constants.dsLeftPositionStrafeDistance);
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
        double heading = direction * Constants.dsRightPositionHeading;
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

  */
}
