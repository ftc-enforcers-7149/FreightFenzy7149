package org.firstinspires.ftc.teamcode.Mattu.ballsnblocks;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

@Config
public class BallsNBlocks extends ImagePipeline {

    //Crop data
    public static int CROP_POS_X = 0;
    public static int CROP_POS_Y = 0;
    public static int CROP_SIZE_X = 640;
    public static int CROP_SIZE_Y = 360;

    private ArrayList<RotatedRect> blocks; //List of detected blocks
    private ArrayList<RotatedRect> normalBlocks; //List of detected normal blocks
    private ArrayList<RotatedRect> tapedBlocks; //List of detected taped blocks
    private ArrayList<Circle> balls; //List of detected balls

    private Bitmap bitmap; //Output image for FTCDashboard

    public BallsNBlocks() {
        blocks = new ArrayList<RotatedRect>();
        tapedBlocks = new ArrayList<RotatedRect>();
        normalBlocks = new ArrayList<RotatedRect>();
        balls = new ArrayList<Circle>();

        bitmap = Bitmap.createBitmap(CROP_SIZE_X, CROP_SIZE_Y, Bitmap.Config.RGB_565);
    }

    @Override
    public Mat processFrame(Mat input) {

        //Crop the input to just see the ground and nothing else
        Rect cropSize = new Rect(CROP_POS_X, CROP_POS_Y, CROP_SIZE_X, CROP_SIZE_Y);
        input = new Mat(input, cropSize);
        Mat output = input.clone();

        //The HSV color format is more useful for detecting specific hues, like the yellow on the
        //blocks, than RGB. It helps with varied lighting conditions.
        Mat hsvImage = input.clone();
        Imgproc.cvtColor(hsvImage, hsvImage, Imgproc.COLOR_RGB2HSV);

        //Slightly blur the image so the colors blend better
        Imgproc.medianBlur(hsvImage, hsvImage, 7);

        /// Detect Blocks ///

        //Lower and upper HSV bounds for the detection
        Scalar lowerBlockColor = new Scalar(12, 120, 90);
        Scalar upperBlockColor = new Scalar(32, 255, 255);

        //"Region of interest"
        Mat roi = new Mat(input.width(), input.height(), input.type());

        //Find all pixels in the specified range for detection
        Core.inRange(hsvImage, lowerBlockColor, upperBlockColor, roi);

        //Find contours (outlines) of anything detected
        ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Imgproc.findContours(roi, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        blocks.clear();
        normalBlocks.clear();
        ArrayList<RotatedRect> possibleTapedBlocks = new ArrayList<RotatedRect>();
        for (MatOfPoint contour : contours) {

            //Find the rectangular bounding box of any objects that are big enough
            if (Imgproc.contourArea(contour) > 250) {
                MatOfPoint2f contour2F = new MatOfPoint2f(contour.toArray());
                RotatedRect rect = Imgproc.minAreaRect(contour2F);
                blocks.add(rect);

                //If the rectangle is fairly square, it does not need to be checked as a taped block
                if (Math.round(rect.size.width / rect.size.height) == 1)
                    normalBlocks.add(rect);
                else
                    possibleTapedBlocks.add(rect);
            }
        }

        tapedBlocks.clear();
        //Compare all pairs of rectangles to see if they form a taped block
        A: for (int i = 0; i < possibleTapedBlocks.size() - 1; i++) {
            RotatedRect rect1 = possibleTapedBlocks.get(i);

            B: for (int j = i + 1; j < possibleTapedBlocks.size(); j++) {
                RotatedRect rect2 = possibleTapedBlocks.get(j);

                //The comparison sees if the long edge of a rectangle (which would be the full edge
                //of the block) is the same as the distance between the two rectangles. In other
                //words it checks if two slivers form a square.
                double avgWidth = (Math.max(rect1.size.width, rect1.size.height) +
                        Math.max(rect2.size.width, rect2.size.height)) / 2;
                double avgShortEdge = (Math.min(rect1.size.width, rect1.size.height) +
                        Math.min(rect2.size.width, rect2.size.height)) / 2;
                double dist = distance(rect1.center, rect2.center) + avgShortEdge;

                if ((dist <= avgWidth * 1.25) && (dist >= avgWidth * 0.75)) {
                    Point center = new Point((rect1.center.x + rect2.center.x) / 2,
                            (rect1.center.y + rect2.center.y) / 2);
                    RotatedRect newRect = new RotatedRect(center, new Size(avgWidth, avgWidth), rect1.angle);
                    tapedBlocks.add(newRect);

                    //Once a rectangle is paired, it does not need to be compared anymore
                    continue A;
                }
                //If the rectangle didn't match anything, it is just a normal block
                else if (j == possibleTapedBlocks.size() - 1) normalBlocks.add(rect2);
            }

            //If the rectangle didn't match anything, it is just a normal block
            normalBlocks.add(rect1);
        }

        //Check for intersections between taped blocks and normal blocks
        for (RotatedRect taped : tapedBlocks) {
            for (int i = 0; i < normalBlocks.size(); i++) {
                RotatedRect normal = normalBlocks.get(i);

                Mat intersectRegion = new Mat();
                int intersect  = Imgproc.rotatedRectangleIntersection(taped, normal, intersectRegion);

                //If there's an intersection, that means one block is being detected on two faces
                //Remove the "normal" face and keep the taped face
                if (intersect >= 2) {
                    normalBlocks.remove(i);
                    blocks.remove(normal);

                    i--;
                }
            }
        }

        //Eliminate overlapping taped blocks
        for (int i = 0; i < tapedBlocks.size() - 1; i++) {
            RotatedRect rect1 = tapedBlocks.get(i);

            for (int j = i + 1; j < tapedBlocks.size(); j++) {
                RotatedRect rect2 = tapedBlocks.get(j);

                Mat intersectRegion = new Mat();
                int intersect  = Imgproc.rotatedRectangleIntersection(rect1, rect2, intersectRegion);

                //If there's an intersection, assume the larger rectangle is more accurate
                //Remove the smaller rectangle from tapedBlocks
                if (intersect >= 2) {
                    if (rect1.size.width > rect2.size.width) {
                        tapedBlocks.remove(j);
                        j--;
                    }
                    else {
                        tapedBlocks.remove(i);
                        i--;
                        break;
                    }
                }
            }
        }

        //Draws all detected blocks. Taped blocks are green and normal blocks are red
        for (RotatedRect taped : tapedBlocks) drawRotatedRect(output, taped, new Scalar(0, 0, 255));
        for (RotatedRect normal : normalBlocks) drawRotatedRect(output, normal, new Scalar(255, 0, 0));

        ////////////////////

        /// Detect Balls ///

        //Get grayscale version of the image
        Mat gray = input.clone();
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

        //Lower and upper HSV bounds for the detection
        Scalar lowerBallColor = new Scalar(0, 0, 80);
        Scalar upperBallColor = new Scalar(75, 100, 255);

        //Only save circles that have an average hsv within a specific range
        balls.clear();
        for (double[] circleArr : circles) {
            Circle circle = new Circle(circleArr);

            //Create an image with only the area inside the circle visible and find the average color
            Mat circleMask = new Mat(hsvImage.rows(), hsvImage.cols(), CvType.CV_8U, Scalar.all(0));
            Imgproc.circle(circleMask, circle.center, (int) circle.radius,
                    new Scalar(255, 255, 255), -1);
            Scalar hsvMean = Core.mean(hsvImage, circleMask);

            //If the average color is within the bounds of lowerBallColor and upperBallColor, save it
            if ((hsvMean.val[0] >= lowerBallColor.val[0] && hsvMean.val[0] <= upperBallColor.val[0]) &&
                (hsvMean.val[1] >= lowerBallColor.val[1] && hsvMean.val[1] <= upperBallColor.val[1]) &&
                (hsvMean.val[2] >= lowerBallColor.val[2] && hsvMean.val[2] <= upperBallColor.val[2])) {

                balls.add(circle);

                Imgproc.circle(output, circle.center, (int) circle.radius, new Scalar(0, 255, 0), 2);
                Imgproc.circle(output, circle.center, 1, new Scalar(0, 255, 0), 3);
            }
        }

        ////////////////////

        //Convert image to bitmap for output
        bitmap = Bitmap.createBitmap(CROP_SIZE_X, CROP_SIZE_Y, Bitmap.Config.RGB_565);
        Utils.matToBitmap(output, bitmap);

        return output;
    }

    private void drawRotatedRect(Mat output, RotatedRect rect, Scalar color) {
        Point[] vertices = new Point[4];
        rect.points(vertices);
        for (int j = 0; j < 4; j++){
            Imgproc.line(output, vertices[j], vertices[(j+1)%4], color);
        }
    }

    private double distance(Point p1, Point p2) {
        return Math.sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y));
    }

    public Bitmap getImage() {
        return bitmap;
    }

    public ArrayList<RotatedRect> getBlocks() {
        return blocks;
    }

    public ArrayList<Circle> getBalls() {
        return balls;
    }
}