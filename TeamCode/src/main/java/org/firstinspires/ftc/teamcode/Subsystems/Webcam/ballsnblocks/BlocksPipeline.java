package org.firstinspires.ftc.teamcode.Subsystems.Webcam.ballsnblocks;

import android.graphics.Bitmap;

import org.firstinspires.ftc.teamcode.Subsystems.Webcam.RectPipeline;
import org.opencv.android.Utils;
import org.opencv.core.Core;
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

public class BlocksPipeline extends RectPipeline {

    //Lower and upper HSV bounds for the detection
    public static Scalar lowerColor = new Scalar(12, 120, 90);
    public static Scalar upperColor = new Scalar(32, 255, 255);

    //Crop data
    public static int CROP_POS_X = 0;
    public static int CROP_POS_Y = 0;
    public static int CROP_SIZE_X = 640;
    public static int CROP_SIZE_Y = 360;

    private final ArrayList<RotatedRect> blocks; //List of detected blocks

    public BlocksPipeline() {
        blocks = new ArrayList<RotatedRect>();
    }

    @Override
    public Mat processFrame(Mat input) {

        //Crop the input to just see the ground and nothing else
        Rect cropSize = new Rect(CROP_POS_X, CROP_POS_Y, CROP_SIZE_X, CROP_SIZE_Y);
        input = new Mat(input, cropSize);

        //The HSV color format is more useful for detecting specific hues, like the yellow on the
        //blocks, than RGB. It helps with varied lighting conditions.
        Mat hsvImage = input.clone();
        Imgproc.cvtColor(hsvImage, hsvImage, Imgproc.COLOR_RGB2HSV);

        //Slightly blur the image so the colors blend better
        Imgproc.medianBlur(hsvImage, hsvImage, 7);

        //"Region of interest"
        Mat roi = new Mat(input.width(), input.height(), input.type());

        //Find all pixels in the specified range for detection
        Core.inRange(hsvImage, lowerColor, upperColor, roi);

        //Find contours (outlines) of anything detected
        ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Imgproc.findContours(roi, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        blocks.clear();
        for (MatOfPoint contour : contours) {

            //Find the rectangular bounding box of any objects that are big enough
            if (Imgproc.contourArea(contour) > 250) {
                MatOfPoint2f contour2F = new MatOfPoint2f(contour.toArray());
                RotatedRect rect = Imgproc.minAreaRect(contour2F);
                blocks.add(rect);
            }
        }

        //Convert image to bitmap for output
        bitmap = Bitmap.createBitmap(CROP_SIZE_X, CROP_SIZE_Y, Bitmap.Config.RGB_565);
        Utils.matToBitmap(input, bitmap);

        hsvImage.release();
        roi.release();

        return input;
    }

    private double distance(Point p1, Point p2) {
        return Math.sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y));
    }

    public ArrayList<RotatedRect> getBlocks() {
        return blocks;
    }
}
