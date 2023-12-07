package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "RedFarCenter528", group = "Autonomous")
public class RedFarCenter528 extends AutoMaster {
    public RedFarCenter528() {
        super(Alliance.RED, StartPosition.FAR, ParkPosition.CENTER);
    }
    @Override
    public void runOpMode() {
        int riggingDirection;
        int boardDirection;
        int parkDirection;
        PropPosition propPosition;
        int targetAprilTagNumber;

        AprilTagProcessor aprilTagProcessor = null;
        VisionPortal visionPortal = null;
        bot = new Bot(this, Constants.maxAutoSpeed);

        riggingDirection = determineRiggingDirection();

        boardDirection = determineBoardDirection(riggingDirection);

        parkDirection = determineParkDirection(parkPosition, boardDirection);

        bot.wristDown();
        sleep(250);
        bot.grabberClose();
        sleep(1000);

        // Raise lift, raise wrist, close grabber
        setToHighCruisingPosition();

        aprilTagProcessor = createAprilTagProcessor();

        visionPortal = createVisionPortal(Constants.atExposureMS, Constants.atExposureGain, aprilTagProcessor);

        sleep(1000);

        waitForStart();

        // Determine prop position, place the purple pixel on the spike mark, then go to escape position
        propPosition = dsPlacePurplePixel(riggingDirection);
        bot.turnToHeading(0);
        bot.moveStraightForDistance(35);
        bot.turnToHeading(boardDirection * -90);
        bot.moveStraightForDistance(96);

        targetAprilTagNumber = aprilTagNumber(propPosition, riggingDirection, boardDirection);

        roughAlignToAprilTag(boardDirection, targetAprilTagNumber, startPosition);

        // Correct strafe to directly face the target April Tag
        autoOrientToAprilTag(aprilTagProcessor, targetAprilTagNumber);

        bot.turnToHeading(boardDirection * -90);

        placePixelOnBoard();

        // Move straight until close to the wall, turn, and parallel park
        park(boardDirection, targetAprilTagNumber, parkDirection);

//        // Lower lift, lower wrist, open grabber
        setStationaryPosition();
    }
}

