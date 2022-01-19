package org.firstinspires.ftc.teamcode.Subsystems.Webcam;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.RotatedRect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class OpenCV {

    private FtcDashboard dashboard;
    private final boolean useDash;

    private OpenCvCamera webcam;
    private RectPipeline pipeline;

    public OpenCV(HardwareMap hardwareMap, FtcDashboard dashboard) {
        this.dashboard = dashboard;
        useDash = true;

        //Use this one to not activate the camera view on the robot controller
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
    }

    public OpenCV(HardwareMap hardwareMap) {
        useDash = false;

        //Use this one to not activate the camera view on the robot controller
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
    }

    public void start(RectPipeline rectPipeline) {
        setPipeline(rectPipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                //Starts streaming with width, height, and rotation
                webcam.startStreaming(640, 360, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    public void setPipeline(RectPipeline rectPipeline) {
        pipeline = rectPipeline;
        webcam.setPipeline(pipeline);
    }

    public void update() {
        //Send an image to the dashboard for debugging
        if (useDash) {
            Bitmap image = pipeline.getImage();
            if (image != null) {
                dashboard.sendImage(image);
            }
        }
    }

    public RotatedRect getRect() {
        if (pipeline != null) return pipeline.getRect();
        else return null;
    }

    /**
     * Stops the camera altogether
     */
    public void stop() {
        webcam.closeCameraDevice();
    }
}
