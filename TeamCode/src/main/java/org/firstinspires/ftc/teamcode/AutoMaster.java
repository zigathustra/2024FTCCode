package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/*
    Program to complete the autonomous phase of the game. This first version assumes the bot starts
    on the red alliance, on the right, closest position when facing the board.
    Steps to complete the challenge:
    1. Determine position of the prop (1, 2, or 3)
    2. Move purple pixel to the spike mark on the selected line (1, 2, or 3)
    3. Set April tag target. 1, 2, or 3 for blue. 4, 5, or 6 for red.
    4. Rotate until you acquire the target April tag.
    5. Navigate to the target April tag.
    6. Place yellow pixel on the board in one of the two slots above the target April tag.
    7. Move to a backstage location that does not interfere with the board.
 */
@TeleOp(name = "AutoMaster", group = "Linear OpMode")
public class AutoMaster extends LinearOpMode {

    // Set alliance
    // 1 = Blue
    // 2 = Red
    final int alliance = 2;
    int targetAprilTag = 0; // Default of no AprilTag target. Will be set after prop detection.
    int targetPropLocation = 2; // Default of prop location 2. Will be set after prop detection.
    // Adjust these numbers to suit your robot.
    final double MAX_VELOCITY = RevMotorInfo.MAX_VELOCITY;
    final double DESIRED_DISTANCE = 10.0; //  this is how close the camera should get to the target (inches)
    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN = 0.03;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN = 0.015;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;
    private Rev2mDistanceSensor distanceSensor;
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    private int myExposure;
    private int myGain;
    private TfodProcessor tfod;

    @Override
    public void runOpMode() {
        // you can use thi
        boolean targetFound = false;    // Set to true when an AprilTag target is detected
        boolean pixelFound = false;
        double drive = 0;        // Desired forward power/speed (-1 to +1)
        double strafe = 0;        // Desired strafe power/speed (-1 to +1)
        double turn = 0;        // Desired turning power/speed (-1 to +1)

        boolean propFound = false;
        int currentStep = 1;
        ElapsedTime runtime = new ElapsedTime();

        initTfod();

        // Initialize the Apriltag Detection process
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "right_back_drive");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distance_sensor");
        initAprilTag();

        setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData("Status", "Initialized. Press run.");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
 /*
            // Get gamepad inputs and set motor power. Used only under TeleOp control
            double max;
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            leftFrontDrive.setVelocity(leftFrontPower * MAX_VELOCITY);
            rightFrontDrive.setVelocity(rightFrontPower * MAX_VELOCITY);
            leftBackDrive.setVelocity(leftBackPower * MAX_VELOCITY);
            rightBackDrive.setVelocity(rightBackPower * MAX_VELOCITY);
            logVelocity();
*/

// Look for prop
// Set target prop location
            targetPropLocation = 2;
// Set target April tag
            targetAprilTag = targetPropLocation;
            if (alliance == 2) {
                targetPropLocation = targetPropLocation * 2;
            }

            //Find the desire April tag
            // Locating desired April tag
            targetFound = false;
            desiredTag = null;
            // STEP 1 move forward
            if (currentStep == 1) {
                if (runtime.milliseconds() < 450) {
                    moveRobot(10, 0, 0);
                } else {
                    moveRobot(0, 0, 0);
                    currentStep = 2;
                    runtime.reset();  // start timer for step 2
                }
            }

            // STEP 2 slight rotation anti-clockwise - look at left spike mark
            if (currentStep == 2) {
                if (runtime.milliseconds() < 175) {
                    moveRobot(0, 0, 5);
                } else {
                    moveRobot(0, 0, 0);
                    currentStep = 3;
                    runtime.reset();  // start timer for step 3
                }
            }

            // STEP 3 use Tensorflow to check for pixel on left spike mark, allow 5 seconds to elapse
            if (currentStep == 3) {
                if (runtime.milliseconds() < 3000) {
                    List<Recognition> currentRecognitions = tfod.getRecognitions();
                    // Step through the list of recognitions and look for pixel
                    for (Recognition recognition : currentRecognitions) {
                        if (recognition.getLabel() == "Pixel") {
                            currentStep = 4;
                            targetPropLocation = 1;
                            pixelFound = true;
                            runtime.reset();  // start timer for step 4
                        } else {
                            sleep(50);
                        }
                    }   // end for() loop
                } else {
                    // pixel not found, try centre spike mark
                    currentStep = 6;
                    runtime.reset();  // start timer for step 6
                }
            }

            // STEP 4 move forward towards left spike mark
            if (currentStep == 4) {
                if (runtime.milliseconds() < 460) {
                    moveRobot(5, 0, 0);
                } else {
                    moveRobot(0, 0, 0);
                    currentStep = 5;
                    runtime.reset();  // start timer for step 5
                }
            }

            // STEP 5 backoff from left spike mark, dropping off purple pixel
            if (currentStep == 5) {
                if (runtime.milliseconds() < 460) {
                    moveRobot(-5, 0, 0);
                } else {
                    moveRobot(0, 0, 0);
                    currentStep = 7;  // point at backdrop
                    runtime.reset();  // start timer for step 7
                }
            }

            // STEP 6 slight rotation clockwise to check centre spike mark
            if (currentStep == 6) {
                if (runtime.milliseconds() < 125) {
                    moveRobot(0, 0, -5);
                } else {
                    moveRobot(0, 0, 0);
                    currentStep = 10;
                    runtime.reset();  // start timer for step 10
                }
            }

            // STEP 7 bigger rotation clockwise from right mark to backdrop
            if (currentStep == 7) {
                if (runtime.milliseconds() < 750) {
                    moveRobot(0, 0, -5);
                } else {
                    moveRobot(0, 0, 0);
                    currentStep = 20;    // head to backdrop
                    //runtime.reset();  // start timer for step 10
                }
            }

            // STEP 10 - use Tensorflow to check for pixel on centre spike mark, allow 5 seconds to elapse
            if (currentStep == 10) {
                if (runtime.milliseconds() < 3000) {
                    List<Recognition> currentRecognitions = tfod.getRecognitions();
                    // Step through the list of recognitions and look for pixel
                    for (Recognition recognition : currentRecognitions) {
                        if (recognition.getLabel() == "Pixel") {
                            currentStep = 11; // drop off at centre
                            targetPropLocation = 2;
                            pixelFound = true;
                            runtime.reset();  // start timer for step 11
                        } else {
                            sleep(50);
                        }
                    }   // end for() loop
                } else {
                    // pixel not found, assume right spike mark
                    targetPropLocation = 3;
                    currentStep = 15; // drop off on right mark
                    runtime.reset();  // start timer
                }
            }

            // STEP 11 - move forward towards centre spike mark
            if (currentStep == 11) {
                if (runtime.milliseconds() < 500) {
                    moveRobot(5, 0, 0);
                } else {
                    moveRobot(0, 0, 0);
                    currentStep = 12;
                    runtime.reset();  // start timer for step 12
                }
            }

            // STEP 12 backoff from centre spike mark, dropping off purple pixel
            if (currentStep == 12) {
                if (runtime.milliseconds() < 500) {
                    moveRobot(-5, 0, 0);
                } else {
                    moveRobot(0, 0, 0);
                    currentStep = 13;    // turn toward backdrop
                    runtime.reset();  // start timer for step 13
                }
            }

            // STEP 13 bigger rotation clockwise from center mark to backdrop
            if (currentStep == 13) {
                if (runtime.milliseconds() < 500) {
                    moveRobot(0, 0, -5);
                } else {
                    moveRobot(0, 0, 0);
                    currentStep = 20;    // head to backdrop
                }
            }

            // STEP 15 - turn towards right spike mark
            if (currentStep == 15) {
                if (runtime.milliseconds() < 200) {
                    moveRobot(0, 0, -5);
                } else {
                    moveRobot(0, 0, 0);
                    currentStep = 16;  // move towards mark
                    runtime.reset();  // start timer
                }
            }

            // STEP 16  move forward towards right spike mark
            if (currentStep == 16) {
                if (runtime.milliseconds() < 480) {
                    moveRobot(5, 0, 0);
                } else {
                    moveRobot(0, 0, 0);
                    currentStep = 17;
                    runtime.reset();  // start timer for step 17
                }
            }

            // STEP 17  move backward from right spike mark
            if (currentStep == 17) {
                if (runtime.milliseconds() < 480) {
                    moveRobot(-5, 0, 0);
                } else {
                    moveRobot(0, 0, 0);
                    currentStep = 18;
                    runtime.reset();  // start timer for step 18
                }
            }

            // STEP 18 - point webcam 2 at background
            if (currentStep == 18) {
                if (runtime.milliseconds() < 250) {
                    moveRobot(0, 0, -5);
                } else {
                    moveRobot(0, 0, 0);
                    currentStep = 20;  // go to backdrop
                    //runtime.reset();  // start timer
                }
            }
            // STEP 20 - use April Tags to move toward backdrop
            //  start by closing TFOD and starting April Tag
            if (currentStep == 20) {
                visionPortal.close();
                sleep(50);
                // Initialize the Apriltag Detection process
                initAprilTag();
                sleep(50);
                setManualExposure(20, 250);  // Use low exposure time high gain to reduce motion blur

                targetAprilTag = targetPropLocation * alliance;

                currentStep = 22;
                //runtime.reset();  // timer for step 21
            }

            // STEP 21 - move to backdrop - not really needed

            // STEP 22 - move to backdrop using April Tag
            if (currentStep == 22) {
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                for (AprilTagDetection detection : currentDetections) {
                    if ((detection.metadata != null) && (detection.id == targetAprilTag)) {
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    }
                }

                if (targetFound) {
                    // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                    double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                    double headingError = desiredTag.ftcPose.bearing;
                    double yawError = desiredTag.ftcPose.yaw;
                    if ((rangeError < 7) && (Math.abs(headingError) < 5) && (Math.abs(yawError) < 5)) {
                        // if we're close enough, stop using April Tag logic
                        drive = 0;
                        turn = 0;
                        strafe = 0;
                        currentStep = 23;  // drive to backdrop
                    } else {
                        // Use the speed and turn "gains" to calculate how we want the robot to move.
                        drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                        turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                        strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                    }
                    telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);

                    // Apply desired axes motions to the drivetrain.
                    moveRobot(drive, strafe, turn);
                    sleep(10);
                } else {
                    moveRobot(0, 0, 0);
                    sleep(10);
                }
            }

            // STEP 23 touch backdrop, drive until touch sensor
       /*     if (currentStep==23) {
                if (touchSensor.isPressed()) {
                    //stop when sensor touched
                    moveRobot(0, 0, 0);
                    currentStep=24;  // deploy arm
                }
                else {
                    moveRobot(0.2, 0, 0);  // move forward slowly
                }
            }



            // STEP 24 deploy arm with yellow pixel to backdrop
            if (currentStep==24) {
                position -= INCREMENT ;
                if (position <= MIN_POS ) {
                    position = MIN_POS;
                    currentStep = 25;  // raise arm
                    sleep(250);        // let pixel fall
                }
                servo.setPosition(position);
                sleep(CYCLE_MS);  // let servo have time to move
            }

            // STEP 25 retract arm
            if (currentStep==25) {
                position += 0.02;
                if (position >= MAX_POS ) {
                    position = MAX_POS;
                    currentStep = 30;  // go park
                    runtime.reset();  // start timer, allow for final sleep cycle
                }
                servo.setPosition(position);
                sleep(CYCLE_MS);  // let servo have time to move
            }

        */
            // STEP 30 - move to the right and park
            if (currentStep == 30) {
                if (runtime.milliseconds() < 500) {
                    moveRobot(0, -5, 0);
                    sleep(10);
                } else { // end of autonomous, robot will do nothing else
                    moveRobot(0, 0, 0);
                    sleep(10);
                }
            }

            telemetry.addData("current step", currentStep);
            telemetry.addData("prop found", pixelFound);
            telemetry.addData("prop location", targetPropLocation);
            telemetry.addData("tag target", targetAprilTag);
            telemetry.addData("tag found", targetFound);
            telemetry.update();
        }
        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((targetAprilTag < 0) || (detection.id == targetAprilTag)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }

        // Tell the driver what we see, and what to do.
        if (targetFound) {
            telemetry.addData("\n>", "HOLD Left-Bumper to Drive to Target\n");
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
        } else {
            telemetry.addData("\n>", "Drive using joysticks to find valid target\n");
        }

        // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
        if (gamepad1.left_bumper && targetFound) {

            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
            double headingError = desiredTag.ftcPose.bearing;
            double yawError = desiredTag.ftcPose.yaw;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        } else {

            // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
            drive = -gamepad1.left_stick_y / 2.0;  // Reduce drive rate to 50%.
            strafe = -gamepad1.left_stick_x / 2.0;  // Reduce strafe rate to 50%.
            turn = -gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.
            telemetry.addData("Manual", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        }
        telemetry.update();

        // Apply desired axes motions to the drivetrain.
        moveRobot(drive, strafe, turn);
        sleep(10);

        // generic DistanceSensor methods.
        telemetry.addData("deviceName", distanceSensor.getDeviceName());
        telemetry.addData("range", String.format("%.01f mm", distanceSensor.getDistance(DistanceUnit.MM)));
        telemetry.addData("range", String.format("%.01f cm", distanceSensor.getDistance(DistanceUnit.CM)));
        telemetry.addData("range", String.format("%.01f m", distanceSensor.getDistance(DistanceUnit.METER)));
        telemetry.addData("range", String.format("%.01f in", distanceSensor.getDistance(DistanceUnit.INCH)));

        // Rev2mDistanceSensor specific methods.
        telemetry.addData("ID", String.format("%x", distanceSensor.getModelID()));
        telemetry.addData("did time out", Boolean.toString(distanceSensor.didTimeoutOccur()));

        telemetry.update();

    }

    private void logVelocity() {
        telemetry.addData("leftFrontDrive (counts/sec): ", "%7d :%7d",
                leftFrontDrive.getVelocity());
        telemetry.addData("leftBackDrive (counts/sec): ", "%7d :%7d",
                leftBackDrive.getVelocity());
        telemetry.addData("rightFrontDrive (counts/sec): ", "%7d :%7d",
                rightFrontDrive.getVelocity());
        telemetry.addData("rightBackDrive (counts/sec): ", "%7d :%7d",
                rightBackDrive.getVelocity());
        telemetry.update();
    }

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()
                // Use setModelAssetName() if the TF Model is built in as an asset.
                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)
                //.setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)
                .build();
        tfod.setZoom(1.5);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1920, 1080));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }

    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower = x + y - yaw;
        double rightBackPower = x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send velocity to the wheels.
        leftFrontDrive.setVelocity(leftFrontPower * MAX_VELOCITY);
        rightFrontDrive.setVelocity(rightFrontPower * MAX_VELOCITY);
        leftBackDrive.setVelocity(leftBackPower * MAX_VELOCITY);
        rightBackDrive.setVelocity(rightBackPower * MAX_VELOCITY);
        logVelocity();
    }

    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}
