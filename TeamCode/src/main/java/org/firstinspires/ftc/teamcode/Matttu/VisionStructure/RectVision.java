package org.firstinspires.ftc.teamcode.Matttu.VisionStructure;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;

public abstract class RectVision extends Vision_Base {

    public RectVision(int posX, int posY, int width, int height) {
        super(posX, posY, width, height);
    }

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

    @Override
    public void init(Mat mat) {
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
}
