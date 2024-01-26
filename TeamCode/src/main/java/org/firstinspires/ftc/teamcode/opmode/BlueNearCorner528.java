package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.Alliance;
import org.firstinspires.ftc.teamcode.common.ParkPosition;
import org.firstinspires.ftc.teamcode.common.StartPosition;
import org.firstinspires.ftc.teamcode.auto.AutoMaster;

@Autonomous(name = "\uD83D\uDCA7BlueNearCorner528", group = "BlueNear")
public class BlueNearCorner528 extends AutoMaster {
    public BlueNearCorner528() {
        super(Alliance.BLUE, StartPosition.NEAR, ParkPosition.CORNER);
    }
}

