package org.firstinspires.ftc.teamcode.test;


import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.common.PropPipeline;
import org.firstinspires.ftc.teamcode.common.Alliance;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "VisionProcessingTest", group = "Test")
public class VisionProcessingTest extends LinearOpMode {

    public void runOpMode() {
//        ElapsedTime runTimer = new ElapsedTime();

//        int propPosition;
//        int targetAprilTagNumber;

        PropPipeline propProcessor = null;
//        AprilTagProcessor aprilTagProcessor = null;
        VisionPortal visionPortal = null;

        propProcessor = createPropProcessor();
        propProcessor.setAlliance(Alliance.RED);

//        aprilTagProcessor = createAprilTagProcessor();


        visionPortal = createVisionPortal(propProcessor);

        visionPortal.setProcessorEnabled(propProcessor, true);
        //       visionPortal.resumeLiveView();
        sleep(100);

        while (!isStopRequested()) {
            if (gamepad1.left_bumper) {
                propProcessor.setAlliance(Alliance.RED);
            } else if (gamepad1.right_bumper) {
                propProcessor.setAlliance(Alliance.BLUE);
            }
            telemetry.addData("Alliance: ", propProcessor.getAlliance());
            telemetry.addData("Prop Direction: ", propProcessor.getPropDirection());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }

    protected PropPipeline createPropProcessor() {
        PropPipeline propProcessor = new PropPipeline();
        return (propProcessor);
    }

    //   Manually set the camera gain and exposure.
    //   This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    protected VisionPortal createVisionPortal(PropPipeline propProcessor) {

        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(propProcessor)
                .enableLiveView(true)
                .setCameraResolution(new Size(640, 480))
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


