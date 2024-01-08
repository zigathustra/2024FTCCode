package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Alliance;
import org.firstinspires.ftc.teamcode.common.Bot;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.ParkPosition;
import org.firstinspires.ftc.teamcode.common.PropDirection;
import org.firstinspires.ftc.teamcode.common.PropPipeline;
import org.firstinspires.ftc.teamcode.common.StartPosition;

import org.firstinspires.ftc.teamcode.common.VisionSensor;
import org.firstinspires.ftc.teamcode.java.auto.AutoMaster;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@Autonomous(name = "BoardPlacementTest", group = "Test")
public class BoardPlacementTest extends AutoMaster {
    public BoardPlacementTest() {
        super(Alliance.RED, StartPosition.NEAR, ParkPosition.CORNER);
    }
    public void runOpMode() {
        int riggingDirection;
        int boardDirection;
        int parkDirection;
        int targetAprilTagNumber;
        PropPipeline propProcessor = null;
        AprilTagProcessor aprilTagProcessor;
        PropDirection propDirection = null;
        ElapsedTime runTimer = new ElapsedTime();

        bot = new Bot(this, Constants.maxAutoSpeed);

        visionSensor = new VisionSensor(this, alliance);

        riggingDirection = determineRiggingDirection();

        boardDirection = determineBoardDirection(riggingDirection);

        parkDirection = determineParkDirection(parkPosition, boardDirection);

        visionSensor.goToPropDetectionMode();

        bot.wristDown();
        bot.grabberClose();
        sleep(500);

        while (!isStarted() && !isStopRequested()) {
            propDirection = visionSensor.getPropDirection();

            telemetry.addData("Prop Position: ", propDirection);
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
        runTimer.reset();
        visionSensor.goToNoSensingMode();
        setToLowCruisingPosition();

        // Move forward to escape position
//        bot.moveStraightForDistance(Constants.pdDistanceToEscapePosition);

        // Place pixel on correct spike mark and return to escape position
        // Use propDirection determined using webcam during init
//        placePropPixel(propDirection, riggingDirection);

//        roughTravelToBoard(boardDirection, riggingDirection);

        visionSensor.goToAprilTagDetectionMode();

//        targetAprilTagNumber = getTargetAprilTagNumber(alliance, propDirection);

//        roughAlignToAprilTag(boardDirection, targetAprilTagNumber, startPosition);

//        if (runTimer.time() <= orientMaxTime()) {
//            // Correct strafe to directly face the target April Tag
//            autoOrientToAprilTag(visionSensor, targetAprilTagNumber, boardDirection);
//        }

        if (runTimer.time() <= placeMaxTime()) {            // Correct strafe to directly face the target April Tag
            placePixelOnBoard();
        }

//        if (runTimer.time() <= parkMaxTime()) {
//            park(boardDirection, targetAprilTagNumber, parkDirection);
//        }
//        telemetry.addData("finish park Time: ",runTimer.time());
//        telemetry.update();
//        sleep(1000);
        // Lower lift, lower wrist, open grabber
        setToTeleopStartingPosition();
    }

    protected void placePixelOnBoard() {
        bot.moveStraightForDistance(Constants.boardApproachDistance);
        bot.strafeForDistance(-Constants.sensorToDrivetrainMiddle);
        bot.liftStopAtPosition(Constants.liftAutoBoardProbePosition);
        bot.wristDown();
//        sleep(500);
        bot.creepUntilContact();
        bot.creepStraightForDistance(-Constants.boardOffsetDistance);
        bot.wristUp();
        bot.liftStopAtPosition(Constants.liftAutoBoardPlacementPosition);
        sleep(250);
        bot.creepStraightForDistance(Constants.boardOffsetDistance + 4.5);
//        bot.liftStopAtPosition(Constants.liftAutoBoardPosition - 150);
        bot.grabberOpen();
        sleep(250);
//        bot.liftStopAtPosition(Constants.liftAutoBoardPosition + 150);
//        sleep(250);
        bot.moveStraightForDistance(-Constants.boardEscapeDistance);
    }

}

