package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="OmniDriveLiftGrabber", group="Linear OpMode")
@Disabled
public class OmniDriveLiftGrabber extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor lift = null;
    private Servo   wrist = null;
    private Servo   grabber = null;

    @Override
    public void runOpMode() {
        boolean tooHigh;
        int maxPosition;
        int minPosition;
        int targetVelocity;
        boolean brakeOn;
        boolean tooLow;

        final double drivePowerFactor = 1.0;

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        wrist =  hardwareMap.get(Servo.class, "wrist");
        grabber =  hardwareMap.get(Servo.class, "grabber");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        lift = hardwareMap.get(DcMotor.class, "lift");
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        maxPosition = 3000;
        minPosition = 0;
        targetVelocity = 1750;
        brakeOn = false;

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  -gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // Send calculated power to wheels
            leftFrontDrive.setPower(drivePowerFactor*leftFrontPower);
            rightFrontDrive.setPower(drivePowerFactor*rightFrontPower);
            leftBackDrive.setPower(drivePowerFactor*leftBackPower);
            rightBackDrive.setPower(drivePowerFactor*rightBackPower);


            if (gamepad1.x) {
                grabber.setPosition(0.5);
            }

            if (gamepad1.a) {
                grabber.setPosition(1.0);
            }

            if (gamepad1.y) {
                wrist.setPosition(0.5);
            }

            if (gamepad1.b) {
                wrist.setPosition(1.0);
            }

            if (brakeOn == false && gamepad1.right_trigger < 0.8 && gamepad1.left_trigger < 0.8) {
                ((DcMotorEx) lift).setVelocity(0);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setTargetPosition(lift.getCurrentPosition());
                lift.setPower(0.75);
                brakeOn = true;
            }
            if (lift.getCurrentPosition() >= maxPosition - 25) {
                tooHigh = true;
                if (brakeOn == false) {
                    ((DcMotorEx) lift).setVelocity(0);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setTargetPosition(maxPosition);
                    lift.setPower(0.75);
                    brakeOn = true;
                }
            } else {
                tooHigh = false;
            }
            if (lift.getCurrentPosition() <= minPosition + 25) {
                tooLow = true;
                if (brakeOn == false) {
                    ((DcMotorEx) lift).setVelocity(0);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setTargetPosition(minPosition);
                    lift.setPower(0.75);
                    brakeOn = true;
                }
            } else {
                tooLow = false;
            }
            if (tooHigh == false && gamepad1.right_trigger > 0.8) {
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                ((DcMotorEx) lift).setVelocity(targetVelocity);
                brakeOn = false;
            }
            if (tooLow == false && gamepad1.left_trigger > 0.8) {
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                ((DcMotorEx) lift).setVelocity(-targetVelocity);
                brakeOn = false;
            }
            sleep(10);
            telemetry.addData("Position", lift.getCurrentPosition());
            telemetry.addData("tooHigh", tooHigh);
            telemetry.addData("tooLow", tooLow);
            telemetry.addData("brakeOn", brakeOn);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }
    }
}
