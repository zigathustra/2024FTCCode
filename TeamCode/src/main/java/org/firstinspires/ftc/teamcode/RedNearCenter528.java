package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedNearCenter528", group = "Autonomous")
public class RedNearCenter528 extends AutoMaster {
    public RedNearCenter528() {
        super(BotPosition.RED_NEAR, ParkPosition.CENTER);
    }

    protected int directionFactor()
    {
        return(1);
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
        double slotsFromParkingPosition = targetAprilTagNumber - 3 + 1.5;
        return(directionFactor * (slotsFromParkingPosition * Constants.distanceBetweenAprilTags));
    }
}

