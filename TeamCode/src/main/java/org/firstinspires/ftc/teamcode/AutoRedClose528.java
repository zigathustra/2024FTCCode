package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Autonomous(name = "AutoRedClose528", group = "Linear OpMode")
@TeleOp(name = "AutoRedClose528", group = "Linear OpMode")

public class AutoRedClose528 extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runTime = null;
    private AutoBot bot = null;
    private int alliance = 2;   // 1=Blue, 2=Red
    private final double objectDistanceThreshold = 10;   // inches

    @Override
    public void runOpMode() {
//        telemetry.addData("Status: ", "Initializing");
//        telemetry.update();
        runTime = new ElapsedTime();
        bot = new AutoBot(this);
        final int liftCruisingPosition = 550;
        final double distanceToMiddleOfSquare = 25;
        int propPosition = 3;
        final double headingOutOfSquare = -135;
        final double distanceOutOfSquare = 20;
        final double parkingHeading1 = -35;
        final double parkingDistance1 = 40;
        final double parkingHeading2 = 0;
        final double parkingDistance2 = 8;
        final double parkingDistance3 = 10;


        //       telemetry.addData("Status: ", "Initialized");
        //       telemetry.update();
        waitForStart();

        //       while (opModeIsActive()) {
//            telemetry.addData("Status: ", "Start of auto");
//            telemetry.update();

        // Raise lift to cruising and sensing height
        bot.wristUp();
        bot.grabberClose();
        bot.liftStopAtPosition(liftCruisingPosition);

        // Move to center of spike marks
        bot.moveStraightForDistance(distanceToMiddleOfSquare);

        propPosition = bot.dsPlacePurplePixel();

//        telemetry.addData("propPosition: ", propPosition);
//        telemetry.update();
//        sleep(3000);

        // Calculate heading of prop relative to last imu reset
        //       propHeading = ((propPosition - 2) * (-90)) + 15;
//        if (propPosition == 2) {
//            bot.turnToHeading(prop2Heading);
//        } else if (propPosition == 3) {
//            bot.turnToHeading(prop3Heading);
//        } else {
//            bot.turnToHeading(prop1Heading);
//            bot.strafeForDistance(-3);
//        }
//        bot.moveStraightForDistance(placementDistance);
//        bot.moveStraightForDistance(-placementDistance);
//        telemetry.addData("propPosition: ", propPosition);
//        telemetry.update();
//        sleep(2000);
        // Turn toward escape corner of red square
        bot.turnToHeading(headingOutOfSquare);

        // Move out of red square
        bot.moveStraightForDistance(distanceOutOfSquare);

        // Turn toward parking position
        bot.turnToHeading(parkingHeading1);

        // Move toward parking position
        bot.moveStraightForDistance(parkingDistance1);
        bot.turnToHeading(parkingHeading2);

        // Move toward parking position
        bot.moveStraightForDistance(parkingDistance2);
        bot.strafeForDistance(parkingDistance3);
        bot.stopDrive();
        // Turn toward board
        // Follow April Tag to board
        // Place pixel

        // Move to parking position
        // Lower lift to position 0!!!!!
    }
    //}
}
