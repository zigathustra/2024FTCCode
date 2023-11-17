package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BlueNearCenter528", group = "Autonomous")
public class BlueNearCenter528 extends AutoMaster {
    public BlueNearCenter528() {
        super(BotPosition.RED_NEAR, ParkPosition.CENTER);
    }

    protected int directionFactor()
    {
        return(-1);
    }

    protected void determineTargetAprilTagNumber() {
        if (propPosition == PropPosition.FAR) {
            targetAprilTagNumber = 3;
        }

        if (propPosition == PropPosition.MIDDLE) {
            targetAprilTagNumber = 2;
        }

        if (propPosition == PropPosition.NEAR) {
            targetAprilTagNumber = 1;
        }
    }
    protected double boardApproachDistance()
    {
        return(2);
    }
    protected double parkStrafeDistance()
    {
        double slotsFromParkingPosition = 4 - targetAprilTagNumber + 2.5;
        return(directionFactor * (slotsFromParkingPosition * Constants.distanceBetweenAprilTags));
    }
}