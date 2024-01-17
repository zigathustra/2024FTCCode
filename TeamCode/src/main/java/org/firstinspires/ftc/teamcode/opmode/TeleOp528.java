package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.TeleOpBot;

@TeleOp(name = "TeleoOp528", group = "Linear OpMode")
public class TeleOp528 extends LinearOpMode {
    private TeleOpBot bot = null;

    @Override
    public void runOpMode() {
        double driveAxial = 0.0;
        double driveStrafe = 0.0;
        double driveYaw = 0.0;
        double leftTrigger = 0.0;
        double rightTrigger = 0.0;

        bot = new TeleOpBot(this);
        bot.wristMiddle();
        waitForStart();

        while (opModeIsActive()) {

//            if (gamepad1.left_bumper) {
//                bot.setToSlowedSpeed();
//            } else {
//                bot.setToFastSpeed();
//            }
            if (gamepad1.dpad_up) {
                bot.creepDirection(1.0, 0.0, 0.0);
            } else if (gamepad1.dpad_down) {
                bot.creepDirection(-1.0, 0.0, 0.0);
            } else if (gamepad1.dpad_left) {
                bot.creepDirection(0.0, 1.0, 0.0);
            } else if (gamepad1.dpad_right) {
                bot.creepDirection(0.0, -1.0, 0.0);
            } else {
                driveAxial = gamepad1.left_stick_y;
                driveStrafe = gamepad1.left_stick_x;
                driveYaw = gamepad1.right_stick_x;
                if ((Math.abs(driveAxial) < 0.2) && (Math.abs(driveStrafe) < 0.2) && (Math.abs(driveYaw) < 0.2)) {
                    bot.stopDrive();
                } else
                    bot.moveDirection(-driveAxial, -driveStrafe, -driveYaw);
            }

            leftTrigger = gamepad1.left_trigger;
            rightTrigger = gamepad1.right_trigger;
            if (leftTrigger > 0.3) {
                bot.liftDown(leftTrigger);
            } else if (rightTrigger > 0.3) {
                bot.liftUp(rightTrigger);
            } else {
                bot.liftStop();
            }

            if (gamepad1.x) {
                bot.grabberClose();
            } else if (gamepad1.a) {
                bot.grabberOpen();
            }

            if (gamepad1.b) {
                bot.wristUp();
            } else if (gamepad1.y) {
                bot.wristMiddle();
            }
            if (gamepad1.share && gamepad1.start) {
                bot.launcherUnlocked();
            } else if (gamepad1.right_bumper && gamepad1.left_bumper) {
                bot.launcherLocked();
            }
        }
    }
}
