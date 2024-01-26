package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.Alliance;
import org.firstinspires.ftc.teamcode.common.ParkPosition;
import org.firstinspires.ftc.teamcode.common.StartPosition;
import org.firstinspires.ftc.teamcode.auto.AutoMaster;

@Autonomous(name = "\uD83D\uDCA7BlueNearCenter528", group = "BlueNear")
public class BlueNearCenter528 extends AutoMaster {
    public BlueNearCenter528() {
        super(Alliance.BLUE, StartPosition.NEAR, ParkPosition.CENTER);
    }
}

