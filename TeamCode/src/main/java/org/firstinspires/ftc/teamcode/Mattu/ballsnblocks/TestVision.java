package org.firstinspires.ftc.teamcode.Mattu.ballsnblocks;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(name = "Test Vision")
@Disabled
public class TestVision extends OpMode {

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private OpenCvCamera webcam;

    private ImagePipeline pipeline;
    public static int pipelineID = 2;
    private int lastPipelineID;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        switch (pipelineID) {
            case 0: pipeline = new Blocks(); break;
            case 1: pipeline = new Balls(); break;
            case 2: pipeline = new BallsNBlocks(); break;
            default: pipeline = null;
        }

        lastPipelineID = pipelineID;

        if (pipeline != null) {
            webcam.setPipeline(pipeline);
        }
        else {
            telemetry.addLine("Unknown pipelineID");
            telemetry.update();
            requestOpModeStop();
        }

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                //Starts streaming with width, height, and rotation
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    @Override
    public void loop() {
        if (pipelineID != lastPipelineID) {
            switch (pipelineID) {
                case 0:
                    pipeline = new Blocks();
                    break;
                case 1:
                    pipeline = new Balls();
                    break;
                case 2:
                    pipeline = new BallsNBlocks();
                    break;
            }
            webcam.setPipeline(pipeline);

            lastPipelineID = pipelineID;
        }

        Bitmap image = pipeline.getImage();
        if (image != null) {
            dashboard.sendImage(image);
        }

        //Telemetry for various pipeline and webcam values
        telemetry.addData("Frame Count", webcam.getFrameCount());
        telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
        telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
        telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
        telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
        telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
    }
}
