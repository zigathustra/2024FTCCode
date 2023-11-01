package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;

public class Lift {
    private DcMotorEx liftMotor = null;
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
            liftMotor.setVelocity(targetSpeed * Constants.liftMaxMoveSpeed * Constants.liftMaxVelocity);
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
            liftMotor.setVelocity(-targetSpeed * Constants.liftMaxMoveSpeed * Constants.liftMaxVelocity);
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
        if (currentPosition > (Constants.liftMaxPosition - Constants.liftMaxTolerance)) {
            stop = true;
            stopAtPosition(Constants.liftMaxPosition);
        }
        return stop;
    }

    private boolean stoppedAtBottom() {
        boolean stop = false;
        int currentPosition = liftMotor.getCurrentPosition();
        if (currentPosition < (Constants.liftMinPosition - Constants.liftminTolerance)) {
            stop = true;
            stopAtPosition(Constants.liftMinPosition);
        }
        return stop;
    }

    public void stopAtPosition(int targetPosition) {
        liftMotor.setTargetPosition(targetPosition);
        liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(Constants.liftStopPowerFactor);
        while (liftMotor.isBusy()){
        }
        brakeOn = true;
    }

    public void moveToPosition(int targetPosition) {
        if ((targetPosition < Constants.liftMaxPosition) && (targetPosition > Constants.liftMinPosition)) {
            stopAtPosition(targetPosition);
        }
    }
}
