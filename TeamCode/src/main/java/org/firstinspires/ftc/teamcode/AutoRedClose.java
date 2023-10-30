package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="AutoRedClose", group = "Autonomous")

public class AutoRedClose extends LinearOpMode {
    // Declare OpMode members.
    private AutoBot528 bot = null;
    private int alliance = 2;   // 1=Blue, 2=Red
    private final double objectDistanceThreshold = 10;   // inches

    @Override
    public void runOpMode() {
        bot = new AutoBot528(this);
        int propPosition = 3;

        waitForStart();

        // Raise lift, raise wrist, close grabber
        bot.setToCruisingPosition();

        // Move to the center of the spike mark square
        bot.moveToCenterOfSquare();

        // Determine prop position, and place the purple pixel on the spike mark
        propPosition = bot.dsPlacePurplePixel();

        // Leave the spike mark square and move to a square that faces the parking position
        bot.escapeSquare(propPosition);

        // Move straight until close to the wall, turn, and parallel park
        bot.moveStraightAndPark();

        // Lower lift, lower wrist, open grabber
        bot.setStationaryPosition();
    }
}
