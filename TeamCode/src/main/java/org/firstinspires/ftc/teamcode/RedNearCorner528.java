package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedNearCorner528", group = "Autonomous")
public class RedNearCorner528 extends AutoMaster {
    public RedNearCorner528() {
        super(BotPosition.RED_NEAR, ParkPosition.CORNER);
    }

    protected int directionFactor()
    {
        return(1);
    }

    protected double boardApproachDistance()
    {
        return(2);
    }

    protected void determineTargetAprilTagNumber() {
        if (propPosition == PropPosition.FAR) {
            targetAprilTagNumber = 4;
        }

        if (propPosition == PropPosition.MIDDLE) {
            targetAprilTagNumber = 5;
        }

        if (propPosition == PropPosition.NEAR) {
            targetAprilTagNumber = 6;
        }
    }

    protected double parkStrafeDistance()
    {
        double slotsFromParkingPosition = -(7 - targetAprilTagNumber + 2.5);
        return(directionFactor * (slotsFromParkingPosition * Constants.distanceBetweenAprilTags));
    }
}

