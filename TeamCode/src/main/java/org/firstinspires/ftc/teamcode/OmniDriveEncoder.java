package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="OmniDriveEncoder", group="Linear OpMode")

public class OmniDriveEncoder extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftRearDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightRearDrive = null;

    @Override
    public void runOpMode() {
        final double MAX_VELOCITY = RevUltra20DcMotorData.maxCountsPerSec;
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        leftRearDrive  = hardwareMap.get(DcMotorEx.class, "left_rear_drive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "right_front_drive");
        rightRearDrive = hardwareMap.get(DcMotorEx.class, "right_rear_drive");

        leftFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotorEx.Direction.REVERSE);

        leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //logVelocity();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double max = 0;

            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftRearPower   = axial - lateral + yaw;
            double rightRearPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftRearPower));
            max = Math.max(max, Math.abs(rightRearPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftRearPower   /= max;
                rightRearPower  /= max;
            }

            leftFrontDrive.setVelocity(leftFrontPower * MAX_VELOCITY);
            rightFrontDrive.setVelocity(rightFrontPower * MAX_VELOCITY);
            leftRearDrive.setVelocity(leftRearPower * MAX_VELOCITY);
            rightRearDrive.setVelocity(rightRearPower * MAX_VELOCITY);
 //           logVelocity();
        }
    }
    private void logVelocity() {
        telemetry.addData("leftFrontDrive (counts/sec): ",  "%7d :%7d",
                leftFrontDrive.getVelocity());
        telemetry.addData("leftRearDrive (counts/sec): ",  "%7d :%7d",
                leftRearDrive.getVelocity());
        telemetry.addData("rightFrontDrive (counts/sec): ",  "%7d :%7d",
                rightFrontDrive.getVelocity());
        telemetry.addData("rightRearDrive (counts/sec): ",  "%7d :%7d",
                rightRearDrive.getVelocity());
        telemetry.update();
    }
}
