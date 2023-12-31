package org.firstinspires.ftc.teamcode.test;

import static android.os.SystemClock.sleep;


import android.graphics.Canvas;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.auto.AutoMaster;
import org.firstinspires.ftc.teamcode.common.Bot;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.PropPosition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "VisionProcessingTest", group = "Test")
public class VisionProcessingTest extends LinearOpMode {


    public void runOpMode() {
//        ElapsedTime runTimer = new ElapsedTime();

//        int propPosition;
//        int targetAprilTagNumber;

        PropProcessor propProcessor = null;
        AprilTagProcessor aprilTagProcessor = null;
        VisionPortal visionPortal = null;

        propProcessor = createPropProcessor();

        aprilTagProcessor = createAprilTagProcessor();

        visionPortal = createVisionPortal(aprilTagProcessor, propProcessor);

        visionPortal.setProcessorEnabled(propProcessor, true);
        visionPortal.setProcessorEnabled(aprilTagProcessor, false);
        visionPortal.resumeLiveView();
        sleep(100);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Prop Position: ", propProcessor.getPosition());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }

        if (!isStopRequested()) {
            setCameraValues(visionPortal, Constants.atExposureMS, Constants.atExposureGain);
            visionPortal.setProcessorEnabled(aprilTagProcessor, true);
            visionPortal.setProcessorEnabled(propProcessor, false);
            visionPortal.stopLiveView();
            sleep(100);
        }

//        runTimer.reset();
    }


    protected PropProcessor createPropProcessor() {
        PropProcessor propProcessor = new PropProcessor();
        return (propProcessor);
    }

    protected AprilTagProcessor createAprilTagProcessor() {
        // Create the AprilTag processor by using a builder.
        AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTagProcessor.setDecimation(2);

        return (aprilTagProcessor);
    }

    //   Manually set the camera gain and exposure.
    //   This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    protected VisionPortal createVisionPortal(
            AprilTagProcessor aprilTagProcessor, PropProcessor propProcessor) {

        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(propProcessor)
                .addProcessor(aprilTagProcessor)
                .build();

        // Make sure camera is streaming
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        return (visionPortal);
    }

    private void setCameraValues(VisionPortal visionPortal, int exposureMS, int gain) {

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
class PropProcessor implements VisionProcessor {
    volatile int position = 2;
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(130,175);
    static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(420,175);
    static final int REGION1_WIDTH = 85;
    static final int REGION1_HEIGHT = 85;
    static final int REGION2_WIDTH = 70;
    static final int REGION2_HEIGHT = 70;

    Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION1_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION1_HEIGHT);
    Point region2_pointA = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x,
            REGION2_TOPLEFT_ANCHOR_POINT.y);
    Point region2_pointB = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x + REGION2_WIDTH,
            REGION2_TOPLEFT_ANCHOR_POINT.y + REGION2_HEIGHT);

    Mat region1_Cb, region2_Cb;
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();
    double avg1, avg2;
    Scalar black = new Scalar(0,0,0);
    Scalar blue = new Scalar(0,0,255);
    Scalar red = new Scalar(255,0 ,0);

    boolean firstCall = true;

    void inputToCb(Mat input)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 2);
    }
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Mat processFrame(Mat input, long captureTimeNanos) {

//        if (firstCall) {
//            firstCall = false;
//
//
//            inputToCb(input);
//            /*
//             * Submats are a persistent reference to a region of the parent
//             * buffer. Any changes to the child affect the parent, and the
//             * reverse also holds true.
//             */
//            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
//            region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
//        }
      /*
         * Compute the average pixel value of each submat region. We're
         * taking the average of a single channel buffer, so the value
         * we need is at index 0. We could have also taken the average
         * pixel value of the 3-channel image, and referenced the value
         * at index 2 here.
         */
//        avg1 = (int) Core.mean(region1_Cb).val[0];
//        avg2 = (int) Core.mean(region2_Cb).val[0];

        /*
         * Draw a rectangle showing sample region 1 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                blue, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        /*
         * Draw a rectangle showing sample region 2 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                region2_pointA, // First point which defines the rectangle
                region2_pointB, // Second point which defines the rectangle
                blue, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

//        /*
//         * Find the max of the 2 averages
//         */
//        double max = Math.max(avg1, avg2);
//
//        /*
//         * Now that we found the max, we actually need to go and
//         * figure out which sample region that value was from
//         */
//        if(max == avg1) // Was it from region 1?
//        {
//            position = 1; // Record our analysis
//
//            /*
//             * Draw a solid rectangle on top of the chosen region.
//             * Simply a visual aid. Serves no functional purpose.
//             */
//            Imgproc.rectangle(
//                    input, // Buffer to draw on
//                    region1_pointA, // First point which defines the rectangle
//                    region1_pointB, // Second point which defines the rectangle
//                    black, // The color the rectangle is drawn in
//                    -1); // Negative thickness means solid fill
//        }
//        else if(max == avg2) // Was it from region 2?
//        {
//            position = 2; // Record our analysis
//
//            /*
//             * Draw a solid rectangle on top of the chosen region.
//             * Simply a visual aid. Serves no functional purpose.
//             */
//            Imgproc.rectangle(
//                    input, // Buffer to draw on
//                    region2_pointA, // First point which defines the rectangle
//                    region2_pointB, // Second point which defines the rectangle
//                    black, // The color the rectangle is drawn in
//                    -1); // Negative thickness means solid fill
//        }

        /*
         * Render the 'input' buffer to the viewport. But note this is not
         * simply rendering the raw camera feed, because we called functions
         * to add some annotations to this buffer earlier up.
         */
        return input;

    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    int getPosition() {
        return (position);
    }
}

