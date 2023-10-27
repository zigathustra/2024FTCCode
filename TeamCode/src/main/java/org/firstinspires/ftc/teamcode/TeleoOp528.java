package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOp528", group = "Linear OpMode")
public class TeleoOp528 extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runTime = null;
    private TeleOpBot bot = null;

    @Override
    public void runOpMode() {
        double driveAxial = 0.0;
        double driveStrafe = 0.0;
        double driveYaw = 0.0;
        double leftTrigger = 0.0;
        double rightTrigger = 0.0;

        telemetry.addData("Status: ", "Initializing");
        telemetry.update();
        runTime = new ElapsedTime();
        bot = new TeleOpBot(this);
        telemetry.addData("Status: ", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                bot.creepDirection(-1.0, 0.0, 0.0);
            } else if (gamepad1.dpad_down) {
                bot.creepDirection(1.0, 0.0, 0.0);
            } else if (gamepad1.dpad_left) {
                bot.creepDirection(0.0, -1.0, 0.0);
            } else if (gamepad1.dpad_right) {
                bot.creepDirection(0.0, 1.0, 0.0);
            } else {
                driveAxial = gamepad1.left_stick_y;
                driveStrafe = gamepad1.left_stick_x;
                driveYaw = gamepad1.right_stick_x;
                bot.moveDirection(gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);
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
                bot.wristDown();
            }

        }
    }

    private void logTelemetry() {
    }

    ;
}
