package org.firstinspires.ftc.teamcode.Subsystems.Webcam;

import android.graphics.Bitmap;

import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class SimpleColorPipeline extends RectPipeline {

    //These scalars are the lower and upper possible values for the color
    private final Scalar lowerBound, upperBound;

    //What color to draw bounding rect
    private final Scalar drawColor;

    private int posX, posY;
    private int width, height;

    SimpleColorPipeline(Scalar lowerBound, Scalar upperBound, Scalar drawColor) {
        this(lowerBound, upperBound, drawColor, 0, 0, 0, 0);
    }

    SimpleColorPipeline(Scalar lowerBound, Scalar upperBound, Scalar drawColor,
                        int posX, int posY, int width, int height) {
        this.lowerBound = lowerBound; this.upperBound = upperBound;
        this.drawColor = drawColor;

        this.posX = posX; this.posY = posY;
        this.width = width; this.height = height;

        //Instantiate the bitmap with the right WIDTH and HEIGHT
        bitmap = Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565);
    }

    @Override
    public Mat processFrame(Mat input) {
        //Mat output = input.clone();

        //Crop image to only the needed portion
        Mat cropped = cropMat(input, posX, posY, width, height);

        //Roi stands for 'region of interest'. For us, the entire image is a region of interest
        //This will later hold the actual color mask
        Mat roiTemp = new Mat(cropped.width(), cropped.height(), cropped.type());

        //We convert the rgb image into hsv to make color masking (further down) easier
        final Mat hsvMat = cropped.clone();
        Imgproc.cvtColor(hsvMat, hsvMat, Imgproc.COLOR_RGB2HSV);

        //This 'blurs' the image. It gets rid of sharp edges, mainly to avoid having problems
        //with random bright or dark spots on the rings
        Imgproc.medianBlur(hsvMat, hsvMat, 15);

        //Perform the color masking
        Core.inRange(hsvMat, lowerBound, upperBound, roiTemp);
        Imgproc.medianBlur(roiTemp, roiTemp, 15);

        //The contours of the mask are the outlines of the white 'blobs' in the mask
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(roiTemp, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        //Set max area as 0, so first rectangle is picked by default
        //We check width in case there are small imperfections in the image
        double maxArea = 0;

        //Loop through each contour from the mask
        for (MatOfPoint contour : contours) {

            //Get the bounding rectangle (x, y, width, height) of the contour
            MatOfPoint2f areaPoints = new MatOfPoint2f(contour.toArray());
            RotatedRect rect = Imgproc.minAreaRect(areaPoints);

            //Override the largest rectangle
            if (rect.size != null && rect.size.width * rect.size.height > maxArea) {
                maxArea = rect.size.width * rect.size.height;
                boundingRect = rect.clone();

                //Draw the rectangle on the original image for output / debugging
                //drawFromCropped(output, boundingRect, drawColor, posX, posY);
            }
        }

        //If no rectangles were found, reset boundingRect to null (nothing)
        if (maxArea == 0) {
            boundingRect = new RotatedRect(); //null;
        }

        //Convert the original image (with drawn rectangle) to a bitmap for output
        //bitmap = Bitmap.createBitmap(output.width(), output.height(), Bitmap.Config.RGB_565);
        //Utils.matToBitmap(output, bitmap);

        //input.release();
        cropped.release();
        roiTemp.release();
        hsvMat.release();

        //return output;
        return input;
    }
}
