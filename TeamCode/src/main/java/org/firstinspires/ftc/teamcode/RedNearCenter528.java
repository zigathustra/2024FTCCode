package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedNearCenter528", group = "Autonomous")
public class RedNearCenter528 extends AutoMaster {
    public RedNearCenter528() {
        super(BotPosition.RED_NEAR);
    }

    protected double boardDirectionFactor()
    {
        return(1);
    }

    protected double riggingDirectionFactor()
    {
        return(-1);
    }

    protected int aprilTagNumber(PropPosition propPosition) {
        int tagNumber =  4;

        if (propPosition == PropPosition.MIDDLE) {
            tagNumber = 5;
        }

        if (propPosition == PropPosition.FAR) {
            tagNumber = 6;
        }
        return(tagNumber);
    }
    protected double parkStrafeVector(int targetAprilTagNumber)
    {
        double slotsFromParkingPosition = targetAprilTagNumber - 3 + .5;
        return(-(slotsFromParkingPosition * Constants.distanceBetweenAprilTags + Constants.sensorToDrivetrainMiddle * 2));
    }
}

