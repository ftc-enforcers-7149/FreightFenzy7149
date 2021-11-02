package org.firstinspires.ftc.teamcode.Subsystems.Webcam;

import android.graphics.Bitmap;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public abstract class RectPipeline extends OpenCvPipeline {

    //For calculating yaw and pitch angles
    private class Fraction {
        private final int numerator, denominator;

        Fraction(long a, long b) {
            numerator = (int) (a / gcd(a, b));
            denominator = (int) (b / gcd(a, b));
        }

        /**
         * @return the greatest common denominator
         */
        private long gcd(long a, long b) {
            return b == 0 ? a : gcd(b, a % b);
        }

        public int getNumerator() {
            return numerator;
        }

        public int getDenominator() {
            return denominator;
        }
    }

    //For calculating yaw and pitch angles
    private double centerX, centerY;
    private long imageWidth, imageHeight;
    private final double fov = 110;
    private double horizontalFocalLength, verticalFocalLength;

    protected RotatedRect boundingRect = new RotatedRect(); //The most recent bounding rectangle
    protected Bitmap bitmap; //The most recent image (cropped) from the camera

    @Override
    public void init(Mat mat) {
        super.init(mat);

        imageWidth = mat.width();
        imageHeight = mat.height();

        centerX = ((double) imageWidth / 2) - 0.5;
        centerY = ((double) imageHeight / 2) - 0.5;

        // pinhole model calculations
        double diagonalView = Math.toRadians(this.fov);
        Fraction aspectFraction = new Fraction(this.imageWidth, this.imageHeight);
        int horizontalRatio = aspectFraction.getNumerator();
        int verticalRatio = aspectFraction.getDenominator();
        double diagonalAspect = Math.hypot(horizontalRatio, verticalRatio);
        double horizontalView = Math.atan(Math.tan(diagonalView / 2) * (horizontalRatio / diagonalAspect)) * 2;
        double verticalView = Math.atan(Math.tan(diagonalView / 2) * (verticalRatio / diagonalAspect)) * 2;
        horizontalFocalLength = this.imageWidth / (2 * Math.tan(horizontalView / 2));
        verticalFocalLength = this.imageHeight / (2 * Math.tan(verticalView / 2));
    }

    /**
     * @return The left-right angle to the detected rectangle
     */
    public final double getYaw() {
        return calculateYaw(centerX);
    }

    /**
     * @param offsetCenterX The center-x of the camera
     * @return The yaw (left-right) angle to the rectangle
     */
    public final double calculateYaw(double offsetCenterX) {
        double targetCenterX;
        if (boundingRect == null) targetCenterX = getCenterofRect(null).x;
        else targetCenterX = getCenterofRect(boundingRect.boundingRect()).x;

        return Math.toDegrees(
                Math.atan((targetCenterX - offsetCenterX) / horizontalFocalLength)
        );
    }

    /**
     * @param offsetCenterY The center-y of the camera
     * @return The pitch (up-down) angle to the rectangle
     */
    public final double calculatePitch(double offsetCenterY) {
        double targetCenterY;
        if (boundingRect == null) targetCenterY = getCenterofRect(null).y;
        else targetCenterY = getCenterofRect(boundingRect.boundingRect()).y;

        return -Math.toDegrees(
                Math.atan((targetCenterY - offsetCenterY) / verticalFocalLength)
        );
    }

    /**
     * @param rect Any given rectangle
     * @return The center point of the rectangle
     */
    public final Point getCenterofRect(Rect rect) {
        if (rect == null) {
            return new Point(centerX, centerY);
        }
        return new Point(rect.x + rect.width / 2.0, rect.y + rect.height / 2.0);
    }

    /**
     * Return the bitmap, which is a copy of the camera image
     * @return
     */
    public final Bitmap getImage() {
        return bitmap;
    }

    /**
     * Get the main (largest) rectangle from the most recent image
     * @return
     */
    public final RotatedRect getRect() {
        //If the bounding rect is null, it will return an error, so we return null
        try {
            return boundingRect;
        }
        catch (Exception e) {
            return null;
        }
    }

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
        Rect cropRect = new Rect(posX, posY, width, height);
        return new Mat(mat, cropRect);
    }

    /**
     * Draws a rotated rectangle properly on a mat
     * @param output The mat image to draw on
     * @param rect The rotated rect to draw
     * @param color The color to draw in
     */
    protected final void drawRotatedRect(Mat output, RotatedRect rect, Scalar color) {
        Point[] vertices = new Point[4];
        rect.points(vertices);
        for (int j = 0; j < 4; j++){
            Imgproc.line(output, vertices[j], vertices[(j+1)%4], color);
        }
    }

    /**
     * Draws a bounding rect from a cropped mat on the correct position in original mat
     * @param output The mat to draw on
     * @param boundingRect The bounding rect to draw
     * @param color The color to draw the outline in
     * @param cropX The x position where the crop starts
     * @param cropY The y position where the crop starts
     */
    protected final void drawFromCropped(Mat output, RotatedRect boundingRect, Scalar color,
                                         int cropX, int cropY) {
        boundingRect.center = new Point(boundingRect.center.x + cropX,
                boundingRect.center.y + cropY);
        drawRotatedRect(output, boundingRect, color);
    }
}
