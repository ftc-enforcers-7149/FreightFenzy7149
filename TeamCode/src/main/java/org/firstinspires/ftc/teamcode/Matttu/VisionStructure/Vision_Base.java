package org.firstinspires.ftc.teamcode.Matttu.VisionStructure;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public abstract class Vision_Base {

    private int posX, posY;
    private int width, height;


    public Vision_Base(int posX, int posY, int width, int height) {
        this.posX = posX; this.posY = posY;
        this.width = width; this.height = height;
    }

    public void init(Mat mat) {}

    public final void processInput(Mat input) {
        //Crop image to only the needed portion
        Mat cropped = cropMat(input, posX, posY, width, height);
        processCroppedInput(cropped);
        cropped.release();
    }
    protected abstract void processCroppedInput(Mat cropped);
    protected abstract void drawOutput(Mat output);

    /**
     * Crops a mat to a certain size and position
     * @param mat Source mat image
     * @param posX X position of top left corner of crop
     * @param posY Y position of top left corner of crop
     * @param width Width of crop
     * @param height Height of crop
     * @return New, cropped mat image
     */
    protected final Mat cropMat(Mat mat, int posX, int posY, int width, int height) {
        return new Mat(mat, new Rect(posX, posY, width, height));
    }

    /**
     * Draws a rotated rectangle properly on a mat
     * @param output The mat image to draw on
     * @param rect The rotated rect to draw
     * @param color The color to draw in
     */
    protected final void drawRotatedRect(Mat output, RotatedRect rect, Scalar color, int thickness) {
        Point[] vertices = new Point[4];
        rect.points(vertices);
        for (int j = 0; j < 4; j++){
            Imgproc.line(output, vertices[j], vertices[(j+1)%4], color, thickness);
        }
    }

    /**
     * Draws a bounding rect from a cropped mat on the correct position in original mat
     * @param output The mat to draw on
     * @param boundingRect The bounding rect to draw
     * @param color The color to draw the outline in
     */
    protected final void drawFromCropped(Mat output, RotatedRect boundingRect, Scalar color, int thickness) {
        boundingRect.center = new Point(boundingRect.center.x + posX,
                boundingRect.center.y + posY);
        drawRotatedRect(output, boundingRect, color, thickness);
    }

    /**
     * Draws a bounding rect from a cropped mat on the correct position in original mat
     * @param output The mat to draw on
     * @param circle The circle to draw
     * @param color The color to draw the outline in
     */
    protected final void drawFromCropped(Mat output,  Circle circle, Scalar color, int thickness) {
        circle.center = new Point(circle.center.x + posX,
                circle.center.y + posY);
        Imgproc.circle(output, circle.center, (int) circle.radius, color, thickness);
    }
}
