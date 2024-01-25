package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.Alliance;
import org.firstinspires.ftc.teamcode.common.Bot;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.ParkPosition;
import org.firstinspires.ftc.teamcode.common.PropDirection;
import org.firstinspires.ftc.teamcode.common.PropPipeline;
import org.firstinspires.ftc.teamcode.common.StartPosition;
import org.firstinspires.ftc.teamcode.common.VisionSensor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class FastNear extends AutoMaster {
    protected FastNear(Alliance alliance) {
        super(alliance, StartPosition.NEAR, ParkPosition.CORNER);
    }

    protected double getMaxSpeed(){
        return(Constants.maxAutoSuperSpeed);
    }
}

