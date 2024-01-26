package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.Alliance;
import org.firstinspires.ftc.teamcode.common.Bot;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.ParkPosition;
import org.firstinspires.ftc.teamcode.common.PropDirection;
import org.firstinspires.ftc.teamcode.common.PropPipeline;
import org.firstinspires.ftc.teamcode.common.StartPosition;
import org.firstinspires.ftc.teamcode.common.VisionSensor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class FastNear extends AutoMaster {
    protected FastNear(Alliance alliance) {
        super(alliance, StartPosition.NEAR, ParkPosition.CORNER);
    }

    protected double getMaxSpeed(){
        return(Constants.maxAutoSuperSpeed);
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
            strafeVector = -strafeVector - 3 - adjustedTagNumber * Constants.distanceBetweenAprilTags;
        }

 //       bot.turnToHeading(boardDirection * 90);
        bot.strafeForDistance(strafeVector);
        bot.moveStraightForDistance(10);
    }
}

