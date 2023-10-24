package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//@Autonomous(name = "AutoRedClose", group = "Linear OpMode")
@TeleOp(name = "AutoRedClose", group = "Linear OpMode")
@Disabled
public class AutoRedClose extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runTime = null;
    private AutoBot bot = null;
    private int alliance = 2;   // 1=Blue, 2=Red

    private final double objectDistanceThreshold = 10;   // inches

    @Override
    public void runOpMode() {
        telemetry.addData("Status: ", "Initializing");
        telemetry.update();
        runTime = new ElapsedTime();
        bot = new AutoBot(this);

        final double liftCruisingHeight = 8;
        final double distanceToMiddleOfRedSquare = 25;
        final double headingOutOfRedSquare = -155;
        final double headingOfBoard = -60;
        final double distanceOutOfSquare = 23;
        final double targetDistanceFromTag = 14.0;
        final double backOutDistance = 6;
        final double parkingHeading = 10;
        final double parkingDistance = 18;
        final double placementDistance = 8;

        int propPosition = 2;   // Default center position
        double propHeading = 0;   // Default center position
        int targetTagNumber = 2 * alliance;   // Default center position
        telemetry.addData("Status: ", "Initialized");
        telemetry.update();
        waitForStart();

//    while (opModeIsActive()) {
        telemetry.addData("Status: ", "Start of auto");
        telemetry.update();
        // Raise lift to cruising and scanning height
        // Move to center of red lines
        bot.moveStraightForDistance(distanceToMiddleOfRedSquare);

        // Determine prop position
        // propPosition = bot.tfDetermineObjectPosition();
        propPosition = bot.dsDetermineObjectPosition(objectDistanceThreshold);
//        telemetry.addData("propPosition: ", propPosition);
//        telemetry.update();
//        sleep(4000);

        // Calculate heading of prop relative to last imu reset
        propHeading = (propPosition - 2) * (-90);
        bot.turnToHeading(propHeading);
        bot.moveStraightForDistance(placementDistance);
        bot.moveStraightForDistance(-placementDistance);

        // Turn toward escape corner of red square
        bot.turnToHeading(headingOutOfRedSquare);

        // Move out of red square
        bot.moveStraightForDistance(distanceOutOfSquare);

        // Turn toward board
        bot.turnToHeading(headingOfBoard);

        bot.moveStraightForDistance(6);

        // Move forward slowly
        targetTagNumber = propPosition + ((alliance-1) * 3);
//        telemetry.addData("targetTagNumber: ", targetTagNumber);
 //       telemetry.update();
 //       sleep(4000);
        bot.autoDriveToAprilTag(targetTagNumber, targetDistanceFromTag);

        bot.moveStraightToObject(2);

        // Strafe to drop alignment
        // Raise lift to drop height
        // Raise wrist to drop angle
        // Creep up to drop distance
        // Open grabber
        // Back away
        bot.moveStraightForDistance(-5);
        // Lower wrist
        // Lower lift to cruising height

        bot.turnToHeading(0);
        // Move to parking position
        bot.moveStraightForDistance(9 + (propPosition * 6));

        bot.turnToHeading(-90);
        bot.moveStraightForDistance(12);
        // Lower lift to position 0!!!!!

        //sleep(10);   // milliseconds
        //      }

    }
    private void logTelemetry(){};
}
