package org.firstinspires.ftc.teamcode.Subsystems.Webcam.ballsnblocks;

import android.graphics.Bitmap;

import org.firstinspires.ftc.teamcode.Subsystems.Webcam.RectPipeline;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

public class BallsPipeline extends RectPipeline {

    //Lower and upper HSV bounds for the detection
    public static Scalar lowerColor = new Scalar(0, 0, 80);
    public static Scalar upperColor = new Scalar(75, 100, 255);

    //Crop data
    public static int CROP_POS_X = 0;
    public static int CROP_POS_Y = 0;
    public static int CROP_SIZE_X = 640;
    public static int CROP_SIZE_Y = 360;

    private ArrayList<Circle> balls; //List of detected balls

    public BallsPipeline() {
        balls = new ArrayList<Circle>();
    }

    @Override
    public Mat processFrame(Mat input) {
        //Crop the input to just see the ground and nothing else
        Rect cropSize = new Rect(CROP_POS_X, CROP_POS_Y, CROP_SIZE_X, CROP_SIZE_Y);
        input = new Mat(input, cropSize);

        //Get grayscale version of the image
        Mat gray = input.clone();
        Imgproc.cvtColor(gray, gray, Imgproc.COLOR_RGB2GRAY);
        Imgproc.medianBlur(gray, gray, 3);

        //Detect circular shapes in the image
        Mat circleMat = new Mat();
        Imgproc.HoughCircles(gray, circleMat, Imgproc.CV_HOUGH_GRADIENT, 1.1, 10,
                80, 50, 10, 500);

        System.out.println(circleMat);

        ArrayList<double[]> circles = new ArrayList<>();

        for (int i = 0; i < circleMat.cols(); i++) {
            circles.add(circleMat.get(0, i));
        }

        //Get an HSV image
        Mat hsvImage = input.clone();
        Imgproc.cvtColor(hsvImage, hsvImage, Imgproc.COLOR_RGB2HSV);
        Imgproc.medianBlur(hsvImage, hsvImage, 7);

        //Only save circles that have an average hsv within a specific range
        balls.clear();
        for (double[] circleArr : circles) {
            Circle circle = new Circle(circleArr);

            //Create an image with only the area inside the circle visible and find the average color
            Mat circleMask = new Mat(hsvImage.rows(), hsvImage.cols(), CvType.CV_8U, Scalar.all(0));
            Imgproc.circle(circleMask, circle.center, (int) circle.radius,
                    new Scalar(255, 255, 255), -1);
            Scalar hsvMean = Core.mean(hsvImage, circleMask);

            //If the average color is within the bounds of lowerColor and upperColor, save it
            if ((hsvMean.val[0] >= lowerColor.val[0] && hsvMean.val[0] <= upperColor.val[0]) &&
                    (hsvMean.val[1] >= lowerColor.val[1] && hsvMean.val[1] <= upperColor.val[1]) &&
                    (hsvMean.val[2] >= lowerColor.val[2] && hsvMean.val[2] <= upperColor.val[2])) {

                balls.add(circle);

                Imgproc.circle(input, circle.center, (int) circle.radius, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, circle.center, 1, new Scalar(0, 255, 0), 3);
            }

            circleMask.release();
        }

        //Convert image to bitmap for output
        bitmap = Bitmap.createBitmap(CROP_SIZE_X, CROP_SIZE_Y, Bitmap.Config.RGB_565);
        Utils.matToBitmap(input, bitmap);

        hsvImage.release();
        gray.release();
        circleMat.release();

        return input;
    }

    public ArrayList<Circle> getBalls() {
        return balls;
    }
}
