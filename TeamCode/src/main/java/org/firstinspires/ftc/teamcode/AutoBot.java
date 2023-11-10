package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class AutoBot extends Bot {

    protected Rev2mDistanceSensor distanceSensor = null;

    public AutoBot(LinearOpMode opMode) {
        super(opMode, Constants.maxAutoSpeed);
        distanceSensor = opMode.hardwareMap.get(Rev2mDistanceSensor.class, "distance_sensor");
    }

    public double getDistance() {
        return distanceSensor.getDistance(DistanceUnit.INCH);
    }

    public void moveStraightToObject(double targetDistance) {
//        double distance = 12;
        double objectDistance = distanceSensor.getDistance(DistanceUnit.INCH);
        opMode.telemetry.addData("Distance: ", objectDistance);
        opMode.telemetry.update();
        opMode.sleep(3000);
//        if (objectDistance < targetDistance) {
//            distance = objectDistance;
//        } else {
//            distance = objectDistance - targetDistance;
//        }
        moveStraightForDistance(objectDistance-targetDistance);
    }


}
