package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "WristTest", group = "Autonomous")
public class WristTest extends AutoMaster {
    public WristTest() {
        super(Alliance.BLUE, StartPosition.NEAR, ParkPosition.CENTER);
    }

    @Override
    public void runOpMode() {
        bot = new Bot(this, Constants.maxAutoSpeed);

        setToCruisingPosition();

    }

    protected void setToCruisingPosition() {
        bot.wristUp();
        sleep(100000);
    }
}
