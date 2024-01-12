package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.common.Alliance;
import org.firstinspires.ftc.teamcode.common.ParkPosition;
import org.firstinspires.ftc.teamcode.common.StartPosition;
import org.firstinspires.ftc.teamcode.common.VisionSensor;

import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class FarNoBoard extends org.firstinspires.ftc.teamcode.auto.AutoMaster {
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
    protected void autoOrientToAprilTag(VisionSensor visionSensor, int targetTagNumber, int boardDirection)
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

