package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class FarNoBoard extends AutoMaster {
    public FarNoBoard(Alliance alliance)
    {
        super(alliance, StartPosition.FAR, ParkPosition.NONE);
    }

    @Override
    protected void roughTravelToBoard(int boardDirection, int rigginDirection)
    {
        bot.moveStraightForDistance(12);
    }

    @Override
    protected void roughAlignToAprilTag(int boardDirection, int targetAprilTagNumber, StartPosition startPosition)
    {
    }

    @Override
    protected void autoOrientToAprilTag(AprilTagProcessor aprilTagProcessor, int targetTagNumber, int boardDirection)
    {
    }
    @Override
    protected void placePixelOnBoard()
    {
    }

    protected void park(double boardDirection, int targetAprilTagNumber, double parkDirection)
    {
    }
}

