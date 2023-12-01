package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "AutoStrafeTest", group = "Autonomous")
public class AutoStrafeTest extends AutoMaster{
    public AutoStrafeTest() {
        super(Alliance.BLUE, StartPosition.NEAR, ParkPosition.CENTER);
    }
        @Override
        public void runOpMode() {
            bot = new Bot(this, Constants.maxAutoSpeed);

            AprilTagProcessor aprilTagProcessor = null;
            VisionPortal visionPortal = null;

            aprilTagProcessor = createAprilTagProcessor();

            visionPortal = createVisionPortal(Constants.atExposureMS, Constants.atExposureGain, aprilTagProcessor);

            //       setManualExposure(Constants.atExposureMS, Constants.atExposureGain);  // Use low exposure time to reduce motion blur

            waitForStart();

            // Raise lift, raise wrist, close grabber
            setToHighCruisingPosition();

            sleep(100);

            // Correct strafe to directly face the target April Tag
            autoOrientToAprilTag(aprilTagProcessor, 5);

        }
}
