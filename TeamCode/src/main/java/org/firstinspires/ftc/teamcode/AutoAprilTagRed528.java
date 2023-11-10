package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name = "AutoAprilTagRed528", group = "Linear OpMode")
public class AutoAprilTagRed528 extends AutoMaster {

public AutoAprilTagRed528(){    
    super(1);
    }
    public void runOpMode() {
        double aprilTagDistance = 10.0;
        double sensorDistance = 0;
        double stafeDistance = 7;
        int targetAprilTagNumber = 4;
        double moveFromBoard = 2;
        int boardLiftHeight = 1000;
        
        bot = new AutoBot(this);
        if (alliance == 2) {
            allianceDirection = -1;
        } else {
            allianceDirection = 1;
        }
        bot.wristUp();
        bot.grabberClose();
        waitForStart();
//        autoDriveToAprilTag(4, aprilTagDistance);
       initAprilTag();

        setManualExposure(Constants.atExposureMS, Constants.atExposureGain);  // Use low exposure time to reduce motion blur

        autoOrientToAprilTag(targetAprilTagNumber);

//        telemetry.addData("Distance", sensorDistance);
//        telemetry.update();
//        sleep(5000);
        bot.moveStraightForDistance(12);
        autoOrientToAprilTag(targetAprilTagNumber);
        bot.strafeForDistance(-stafeDistance);
        sensorDistance = bot.getDistance();
        bot.moveStraightForDistance(sensorDistance - 8);
        bot.liftStopAtPosition(boardLiftHeight);
        sensorDistance = bot.getDistance();
        bot.creepStraightForDistance(sensorDistance - 10);
        sleep(500);
        bot.grabberOpen();
        sleep(500);
        bot.moveStraightForDistance(-moveFromBoard);
    }
}