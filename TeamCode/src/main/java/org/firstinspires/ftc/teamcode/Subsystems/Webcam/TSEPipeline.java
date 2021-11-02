package org.firstinspires.ftc.teamcode.Subsystems.Webcam;

import org.opencv.core.Scalar;

public class TSEPipeline extends SimpleColorPipeline {

    //These scalars are the lower and upper possible values for the TSE color
    public static Scalar lowerBound = new Scalar(0, 0, 0);
    public static Scalar upperBound = new Scalar(0, 0, 0);

    //What color to draw bounding rect
    public static Scalar drawColor = new Scalar(0, 0, 0);

    TSEPipeline(int posX, int posY, int width, int height) {
        super(lowerBound, upperBound, drawColor, posX, posY, width, height);
    }
}
