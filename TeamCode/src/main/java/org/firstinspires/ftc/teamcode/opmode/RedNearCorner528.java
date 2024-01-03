package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.Alliance;
import org.firstinspires.ftc.teamcode.common.ParkPosition;
import org.firstinspires.ftc.teamcode.common.StartPosition;

import org.firstinspires.ftc.teamcode.java.auto.AutoMaster;


@Autonomous(name = "RedNearCorner528", group = "RedNear")
public class RedNearCorner528 extends AutoMaster {
    public RedNearCorner528() {
        super(Alliance.RED, StartPosition.NEAR, ParkPosition.CORNER);
    }
}

