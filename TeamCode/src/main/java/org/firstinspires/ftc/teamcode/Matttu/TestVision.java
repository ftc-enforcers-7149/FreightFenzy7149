package org.firstinspires.ftc.teamcode.Matttu;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Matttu.BallsNBlocks.Balls;
import org.firstinspires.ftc.teamcode.Matttu.BallsNBlocks.Blocks;
import org.firstinspires.ftc.teamcode.Matttu.VisionStructure.MultiPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Test Vision")
@Disabled
public class TestVision extends OpMode {

    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private OpenCvCamera webcam;

    private MultiPipeline ballsNBlocks;
    private Balls balls;
    private Blocks blocks;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        ballsNBlocks = new MultiPipeline();
        balls = new Balls();
        blocks = new Blocks();
        ballsNBlocks.addPipeline(balls, blocks);
        webcam.setPipeline(ballsNBlocks);

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
        Bitmap image = ballsNBlocks.getImage();
        if (image != null) dashboard.sendImage(image);

        telemetry.addLine("Balls");
        telemetry.addData("Total: ", balls.getBalls().size());
        telemetry.addData("Largest: ", balls.getLargestBall());

        telemetry.addLine("\nBlocks");
        telemetry.addData("Total: ", blocks.getBlocks().size());
        telemetry.addData("Largest: ", blocks.getLargestBlock());
    }
}
