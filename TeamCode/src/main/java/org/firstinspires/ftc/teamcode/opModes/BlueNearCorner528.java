package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.Alliance;
import org.firstinspires.ftc.teamcode.common.ParkPosition;
import org.firstinspires.ftc.teamcode.common.StartPosition;
import org.firstinspires.ftc.teamcode.auto.AutoMaster;

@Autonomous(name = "BlueNearCorner528", group = "BlueNear")
public class BlueNearCorner528 extends AutoMaster {
    public BlueNearCorner528() {
        super(Alliance.BLUE, StartPosition.NEAR, ParkPosition.CORNER);
    }
}

