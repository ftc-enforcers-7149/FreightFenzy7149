package org.firstinspires.ftc.teamcode.Matttu.VisionStructure;

import android.graphics.Bitmap;

import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public abstract class Pipeline_Base extends OpenCvPipeline {

    private Bitmap bitmap; //The most recent image (cropped) from the camera

    @Override
    public final void init(Mat mat) {
        super.init(mat);

        //Instantiate the bitmap with the right WIDTH and HEIGHT
        bitmap = Bitmap.createBitmap(mat.width(), mat.height(), Bitmap.Config.RGB_565);

        initializeVision(mat);
    }
    public abstract void initializeVision(Mat mat);

    @Override
    public final Mat processFrame(Mat input) {
        Mat output = input.clone();

        processInput(input);
        input.release();

        drawOutput(output);

        //Convert the original image (with drawn rectangle) to a bitmap for output
        bitmap = Bitmap.createBitmap(output.width(), output.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(output, bitmap);

        return output;
    }

    protected abstract void processInput(Mat input);
    protected abstract void drawOutput(Mat output);

    /**
     * Return the bitmap, which is a copy of the camera image
     * @return
     */
    public final Bitmap getImage() {
        return bitmap;
    }
}
