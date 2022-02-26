package org.firstinspires.ftc.teamcode.Subsystems.Webcam.ballsnblocks;

import android.graphics.Bitmap;

import org.firstinspires.ftc.teamcode.Subsystems.Webcam.RectPipeline;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.Size;

public class BallsNBlocksPipeline extends RectPipeline {

    public BallsPipeline ballsPipeline;
    public BlocksPipeline blocksPipeline;

    public BallsNBlocksPipeline() {
        ballsPipeline = new BallsPipeline();
        blocksPipeline = new BlocksPipeline();
    }

    @Override
    public void init(Mat mat) {
        super.init(mat);

        ballsPipeline.init(mat);
        blocksPipeline.init(mat);
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat ballsOutput = ballsPipeline.processFrame(input);
        Mat blocksOutput = blocksPipeline.processFrame(input);

        Mat output = new Mat();
        output.push_back(ballsOutput);
        output.push_back(Mat.zeros(new Size(10, ballsOutput.height()), ballsOutput.type()));
        output.push_back(blocksOutput);

        input.release();
        ballsOutput.release();
        blocksOutput.release();

        bitmap = Bitmap.createBitmap(output.width(), output.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(output, bitmap);

        return output;
    }
}
