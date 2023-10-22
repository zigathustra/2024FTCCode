package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "AutoRedClose", group = "Linear OpMode")

public class AutoRedClose extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runTime = null;
    private AutoRobot bot = null;
    private int alliance = 2;   // 1=Blue, 2=Red

    private final double objectDistanceThreshold = 12;   // inches

    @Override
    public void runOpMode() {
        telemetry.addData("Status: ", "Initializing");
        telemetry.update();
        runTime = new ElapsedTime();
        bot = new AutoRobot(this);

        final double liftCruisingHeight = 8;
        final double distanceToMidddleOfRedSquare = 18;
        final double headingOutOfRedSquare = 45;
        final double headingOfBoard = 90;
        final double distanceOutOfSquare = 18;
        final double creepSpeed = 0.25;
        final double targetDistanceFromTag = 8.0;
        final double parkingHeading = 0;
        final double parkingDistance = 18;

        int propPosition = 2;   // Default center position
        double propHeading = 0;   // Default center position
        int targetTagNumber = 2 * alliance;   // Default center position
        telemetry.addData("Status: ", "Initialized");
        telemetry.update();
        waitForStart();

//        while (opModeIsActive()) {
            telemetry.addData("Status: ", "Start of auto");
            telemetry.update();
            bot.testCall();
//            bot.dsTestDetection();

            telemetry.addData("Status: ", "End of auto");
            telemetry.update();
            // Raise lift to cruising and scanning height
            // Move to center of red lines
            //       bot.moveStraightForDistance(distanceToMidddleOfRedSquare);

            // Determine prop position
            // propPosition = bot.tfDetermineObjectPosition();
//        propPosition = bot.dsDetermineObjectPosition(objectDistanceThreshold);

            // Calculate heading of prop relative to last imu reset
//        propHeading = (propPosition - 2) * 90;
//        bot.turnToHeading(propHeading);
//        bot.moveStraightToObjectAndBack();

            // Turn toward escape corner of red square
//        bot.turnToHeading(headingOutOfRedSquare);

            // Move out of red square
//        bot.moveStraightForDistance(distanceOutOfSquare);

            // Turn toward board
//        bot.turnToHeading(headingOfBoard);

            // Move forward slowly
//        bot.moveDirection(creepSpeed, 0, 0);

//        targetTagNumber = propPosition * alliance;
//        bot.autoDriveToAprilTag(targetTagNumber, targetDistanceFromTag);

            // Strafe to drop alignment
            // Raise lift to drop height
            // Raise wrist to drop angle
            // Creep up to drop distance
            // Open grabber
            // Back away
            // Lower wrist
            // Lower lift to cruising height

//        bot.turnToHeading(parkingHeading);
            // Move to parking position
//        bot.moveStraightForDistance(parkingDistance);

            // Lower lift to position 0!!!!!

            //sleep(10);   // milliseconds
  //      }

    }
}
