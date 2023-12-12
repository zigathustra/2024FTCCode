package org.firstinspires.ftc.teamcode;

public class FarBoard extends AutoMaster {

    protected FarBoard(Alliance alliance)
    {
        super(alliance, StartPosition.FAR, ParkPosition.NONE);
    }
    protected void roughTravelToBoard(int boardDirection, int rigginDirection)
    {
        bot.turnToHeading(0);
        bot.moveStraightForDistance(35);
        bot.turnToHeading(boardDirection * -90);
        bot.moveStraightForDistance(96);
    }

    protected void park(double boardDirection, int targetAprilTagNumber, double parkDirection)
    {
        bot.moveStraightForDistance(6);
    }
}

