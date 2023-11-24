package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BlueNearCenter528", group = "Autonomous")
public class BlueNearCenter528 extends AutoMaster {
    public BlueNearCenter528() {
        super(BotPosition.BLUE_NEAR);
    }

    protected double boardDirectionFactor()
    {
        return(-1);
    }

    protected double riggingDirectionFactor()
    {
        return(1);
    }

    protected int aprilTagNumber(PropPosition propPosition) {
        int tagNumber =  6;

        if (propPosition == PropPosition.MIDDLE) {
            tagNumber = 5;
        }

        if (propPosition == PropPosition.FAR) {
            tagNumber = 4;
        }
        return(tagNumber);
    }
    protected double parkStrafeVector(int targetAprilTagNumber)
    {
        double slotsFromParkingPosition = 7 - targetAprilTagNumber + .5;
        return(slotsFromParkingPosition * Constants.distanceBetweenAprilTags + Constants.sensorToDrivetrainMiddle * 2);
    }
}

