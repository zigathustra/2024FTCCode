package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;

public class Lift {
    private DcMotorEx liftMotor = null;
    private double movePowerFactor = 0.7;

    private double stopPowerFactor = 0.25;
    private double maxVelocity = GoBilda312DcMotorData.maxCountsPerSec;

    private int maxPosition = 2500;
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

    public void move(double targetSpeed) {
        if (!stoppedAtLimit()) {
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftMotor.setVelocity(targetSpeed * movePowerFactor * maxVelocity);
        }
    }
    private boolean stoppedAtLimit() {
        boolean stop = false;
        int currentPosition = liftMotor.getCurrentPosition();
        int targetPosition = currentPosition;
        if (currentPosition > (maxPosition - maxTolerance)) {
            stop = true;
            targetPosition = maxPosition;
        } else if (currentPosition < (minPosition + minTolerance)) {
            stop = true;
            targetPosition = minPosition;
        }
        if (stop) {
            stopAtPosition(targetPosition);
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
