package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="AutoRedClose", group="Iterative OpMode")

public class AutoRedClose extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runTime = null;
    private Robot bot = null;

    @Override
    public void init() {
        runTime = new ElapsedTime();
        bot = new Robot(this);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        runTime.reset();
    }

    @Override
    public void loop() {
        byte propPosition = 0;
        double propHeading = 0;
        // Move to center of red lines
        bot.moveStraightForDistance(18);
        // Locate prop
        propPosition = bot.determineObjectPosition();
        // Calculate heading of prop relative to last imu reset
        propHeading = (propPosition - 2) * 90;
        bot.turnToHeading(propHeading);
        bot.moveStraightToObject();
        bot.moveStraightForDistance(-5);
        // Back up
        // Turn toward board
        // Move forward until April tag acquired
        // Navigate to April tag
        // Place pixel on board
        // Move to parking position
        //sleep(10);   // milliseconds
    }

    @Override
    public void stop() {
    }

}
