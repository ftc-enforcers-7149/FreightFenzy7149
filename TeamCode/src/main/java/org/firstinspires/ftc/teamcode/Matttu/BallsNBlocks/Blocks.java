package org.firstinspires.ftc.teamcode.Matttu.BallsNBlocks;

import org.firstinspires.ftc.teamcode.Matttu.VisionStructure.RectVision;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

public class Blocks extends RectVision {

    private ArrayList<RotatedRect> blocks; //List of detected blocks
    private RotatedRect largest; //The largest block

    public Blocks() {
        this(0, 0, 640, 360);
    }

    public Blocks(int posX, int posY, int width, int height) {
        super(posX, posY, width, height);

        blocks = new ArrayList<RotatedRect>();
        largest = new RotatedRect();
    }

    @Override
    protected void processCroppedInput(Mat cropped) {
        //The HSV color format is more useful for detecting specific hues, like the yellow on the
        //blocks, than RGB. It helps with varied lighting conditions.
        Mat hsvImage = cropped.clone();
        Imgproc.cvtColor(hsvImage, hsvImage, Imgproc.COLOR_RGB2HSV);

        //Slightly blur the image so the colors blend better
        Imgproc.medianBlur(hsvImage, hsvImage, 7);

        //Lower and upper HSV bounds for the detection
        Scalar lowerColor = new Scalar(12, 120, 90);
        Scalar upperColor = new Scalar(32, 255, 255);

        //"Region of interest"
        Mat roi = new Mat(cropped.width(), cropped.height(), cropped.type());

        //Find all pixels in the specified range for detection
        Core.inRange(hsvImage, lowerColor, upperColor, roi);

        //Find contours (outlines) of anything detected
        ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Imgproc.findContours(roi, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        blocks.clear();
        largest = new RotatedRect();
        for (MatOfPoint contour : contours) {
            //Find the rectangular bounding box of any objects that are big enough
            if (Imgproc.contourArea(contour) > 250) {
                MatOfPoint2f contour2F = new MatOfPoint2f(contour.toArray());
                RotatedRect rect = Imgproc.minAreaRect(contour2F); contour2F.release();
                blocks.add(rect);

                if (rect.size.area() > largest.size.area()) largest = rect;
            }

            contour.release();
        }

        hsvImage.release();
        roi.release();
    }

    @Override
    protected void drawOutput(Mat output) {
        for (RotatedRect block : blocks) drawFromCropped(output, block, new Scalar(255, 0, 0), 2);
    }

    public ArrayList<RotatedRect> getBlocks() {
        return blocks;
    }

    public RotatedRect getLargestBlock() {
        return largest;
    }
}
