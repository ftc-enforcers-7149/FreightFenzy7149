package org.firstinspires.ftc.teamcode.Matttu.BallsNBlocks;

import org.firstinspires.ftc.teamcode.Matttu.VisionStructure.Circle;
import org.firstinspires.ftc.teamcode.Matttu.VisionStructure.RectVision;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

public class Balls extends RectVision {

    private ArrayList<Circle> balls; //List of detected balls
    private Circle largest; //The largest ball

    public Balls() {
        this(0, 0, 640, 360);
    }

    public Balls(int posX, int posY, int width, int height) {
        super(posX, posY, width, height);

        balls = new ArrayList<Circle>();
        largest = new Circle();
    }

    @Override
    protected void processCroppedInput(Mat cropped) {
        //Get grayscale version of the image
        Mat gray = cropped.clone();
        Imgproc.cvtColor(gray, gray, Imgproc.COLOR_RGB2GRAY);
        Imgproc.medianBlur(gray, gray, 3);

        //Detect circular shapes in the image
        Mat circleMat = new Mat();
        Imgproc.HoughCircles(gray, circleMat, Imgproc.CV_HOUGH_GRADIENT, 1.1, 10,
                80, 50, 10, 500);

        ArrayList<double[]> circles = new ArrayList<>();

        for (int i = 0; i < circleMat.cols(); i++) {
            circles.add(circleMat.get(0, i));
        }

        //Get an HSV image
        Mat hsvImage = cropped.clone();
        Imgproc.cvtColor(hsvImage, hsvImage, Imgproc.COLOR_RGB2HSV);
        Imgproc.medianBlur(hsvImage, hsvImage, 7);

        //Lower and upper HSV bounds for the detection
        Scalar lowerColor = new Scalar(0, 0, 80);
        Scalar upperColor = new Scalar(75, 100, 255);

        //Only save circles that have an average hsv within a specific range
        balls.clear();
        largest = new Circle();
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
                if (circle.radius > largest.radius) largest = circle;
            }
        }

        gray.release();
        circleMat.release();
        hsvImage.release();
    }

    @Override
    protected void drawOutput(Mat output) {
        for (Circle circle : balls) {
            drawFromCropped(output, circle, new Scalar(0, 255, 0), 2);
            drawFromCropped(output, new Circle(circle.center, 1), new Scalar(0, 255, 0), 3);
        }
    }

    public ArrayList<Circle> getBalls() {
        return balls;
    }

    public Circle getLargestBall() {
        return largest;
    }
}
