package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.Alliance;
import org.firstinspires.ftc.teamcode.common.ParkPosition;
import org.firstinspires.ftc.teamcode.common.StartPosition;

import org.firstinspires.ftc.teamcode.auto.AutoMaster;


@Autonomous(name = "RedNearCenter528", group = "RedNear")
public class RedNearCenter528 extends AutoMaster {
    public RedNearCenter528() {
        super(Alliance.RED, StartPosition.NEAR, ParkPosition.CENTER);
    }
}

