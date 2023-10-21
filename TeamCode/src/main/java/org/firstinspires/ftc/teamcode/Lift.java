package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Lift {
    private DcMotorEx liftMotor = null;
    private double powerFactor = 0.7; // Default of 0.7
    private double maxVelocity = 3000; //Default of 3000

    private double maxPosition = 3500;

    private double minPosition = 0;

    private OpMode opMode = null;
    public Lift(OpMode opMode, double powerFactor){
        this.opMode = opMode;
        this.powerFactor = powerFactor;
        maxVelocity = GoBilda312DcMotorData.maxCountsPerSec;
        liftMotor = opMode.hardwareMap.get(DcMotorEx.class, "lift");

        liftMotor.setDirection(DcMotor.Direction.REVERSE);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void moveUp(){

    }
    public void moveDown(){

    }
    public void setPowerFactor(double powerFactor){
        this.powerFactor = powerFactor;
    }
}
