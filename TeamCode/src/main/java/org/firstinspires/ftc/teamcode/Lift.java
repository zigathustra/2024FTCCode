package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;

public class Lift {
    private DcMotorEx liftMotor = null;
    private double movePowerFactor = 1.0;
    private double stopPowerFactor = 1.0;
    private double maxVelocity = GoBilda312DcMotorData.maxCountsPerSec;
    private int maxPosition = 3000;
    private int maxTolerance = 25;

    private int minPosition = 0;

    private int minTolerance = 25;
    private boolean brakeOn = false;

    private LinearOpMode opMode = null;

    public Lift(LinearOpMode opMode) {
        this.opMode = opMode;

        liftMotor = opMode.hardwareMap.get(DcMotorEx.class, "lift");
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void stop() {
        stopAtPosition(liftMotor.getCurrentPosition());
    }

    public void liftUp(double targetSpeed) {
        if (!stoppedAtTop()) {
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftMotor.setVelocity(targetSpeed * movePowerFactor * maxVelocity);
            opMode.telemetry.addData("Stopped at Top: ", "true");
        } else {
            opMode.telemetry.addData("Stopped at Top: ", "false");
        }
        opMode.telemetry.addData("Position:  ", liftMotor.getCurrentPosition());
        opMode.telemetry.update();

    }

    public void liftDown(double targetSpeed) {
        if (!stoppedAtBottom()) {
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftMotor.setVelocity(-targetSpeed * movePowerFactor * maxVelocity);
            opMode.telemetry.addData("Stopped at Bottom: ", " true");
        } else {
            opMode.telemetry.addData("Stopped at Bottom: ", " false");
        }
        opMode.telemetry.addData("Position:  ", liftMotor.getCurrentPosition());
        opMode.telemetry.update();
    }

    private boolean stoppedAtTop() {
        boolean stop = false;
        int currentPosition = liftMotor.getCurrentPosition();
        if (currentPosition > (maxPosition - maxTolerance)) {
            stop = true;
            stopAtPosition(maxPosition);
        }
        return stop;
    }

    private boolean stoppedAtBottom() {
        boolean stop = false;
        int currentPosition = liftMotor.getCurrentPosition();
        if (currentPosition < (minPosition - minTolerance)) {
            stop = true;
            stopAtPosition(minPosition);
        }
        return stop;
    }

    private void stopAtPosition(int targetPosition) {
        liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        liftMotor.setTargetPosition(targetPosition);
        liftMotor.setPower(stopPowerFactor);
        brakeOn = true;
    }

    public void moveToPosition(int targetPosition) {
        if ((targetPosition < maxPosition) && (targetPosition > minPosition)) {
            stopAtPosition(targetPosition);
        }
    }
}
