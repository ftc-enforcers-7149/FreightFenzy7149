package org.firstinspires.ftc.teamcode.Mattu.ballsnblocks;

import android.graphics.Bitmap;

import org.openftc.easyopencv.OpenCvPipeline;

public abstract class ImagePipeline extends OpenCvPipeline {
    abstract Bitmap getImage();
}
