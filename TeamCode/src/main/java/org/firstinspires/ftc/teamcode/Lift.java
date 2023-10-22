package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;

public class Lift {
    private DcMotorEx liftMotor = null;
    private double powerFactor = 0.75; // Default of 0.7
    private double maxVelocity = 3000; //Default of 3000

    private int maxPosition = 2500;

    private int minPosition = 0;

    private boolean brakeOn = false;

    private LinearOpMode opMode = null;

    public Lift(LinearOpMode opMode, double powerFactor) {
        this.opMode = opMode;
        this.powerFactor = powerFactor;
        maxVelocity = GoBilda312DcMotorData.maxCountsPerSec;

        liftMotor = opMode.hardwareMap.get(DcMotorEx.class, "lift");
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void stop(){
        stopAtPosition(liftMotor.getCurrentPosition());
    }
    public void setPowerFactor(double powerFactor) {
        this.powerFactor = powerFactor;
    }

    public void move(double targetSpeed) {
        if (!stoppedAtLimit()){
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftMotor.setVelocity(targetSpeed * powerFactor * maxVelocity);
        }
    }
    public void moveToPosition(int targetPosition) {
        if ((targetPosition < maxPosition) && (targetPosition > minPosition)){
            stopAtPosition(targetPosition);
        }
    }

    private boolean stoppedAtLimit(){
        boolean stop = false;
        int currentPosition = liftMotor.getCurrentPosition();
        int targetPosition = currentPosition;
        if (currentPosition > (maxPosition - 25)){
            stop = true;
            targetPosition = maxPosition;
        } else if (currentPosition < (minPosition + 25)) {
            stop = true;
            targetPosition = minPosition;
        }
        if (stop){
            stopAtPosition(targetPosition);
        }
        return stop;
    }
    private void stopAtPosition(int targetPosition){
        liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        liftMotor.setTargetPosition(targetPosition);
        liftMotor.setPower(powerFactor);
        brakeOn = true;
    }
}
