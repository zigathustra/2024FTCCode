package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BlueNearCorner528", group = "Autonomous")
public class BlueNearCorner528 extends AutoMaster {
    public BlueNearCorner528() {
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
        double slotsFromParkingPosition = targetAprilTagNumber - 3 + .5;
        return(-(slotsFromParkingPosition * Constants.distanceBetweenAprilTags + Constants.sensorToDrivetrainMiddle * 2));
    }
}

