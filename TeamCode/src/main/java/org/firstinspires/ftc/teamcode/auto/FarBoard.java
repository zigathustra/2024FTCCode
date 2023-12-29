package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.common.Alliance;
import org.firstinspires.ftc.teamcode.common.ParkPosition;
import org.firstinspires.ftc.teamcode.common.StartPosition;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class FarBoard extends AutoMaster {

    protected FarBoard(Alliance alliance)
    {
        super(alliance, StartPosition.FAR, ParkPosition.NONE);
    }

    protected double orientMaxTime() {
        return (30);
    }

    protected double placeMaxTime() {
        return (25);
    }

    protected double parkMaxTime() {
        return (28);
    }
    protected void roughTravelToBoard(int boardDirection, int rigginDirection)
    {
        bot.turnToHeading(0);
        bot.moveStraightForDistance(35);
        bot.turnToHeading(boardDirection * -90);
        bot.moveStraightForDistance(98);
    }

    protected void autoOrientToAprilTag(AprilTagProcessor aprilTagProcessor, int targetTagNumber, int boardDirection)
    {
    }
    protected void park(double boardDirection, int targetAprilTagNumber, double parkDirection)
    {
        bot.moveStraightForDistance(6);
    }
}

