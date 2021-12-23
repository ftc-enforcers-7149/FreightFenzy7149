package org.firstinspires.ftc.teamcode.Matttu.VisionStructure;

import org.opencv.core.Mat;

public class SinglePipeline extends Pipeline_Base {

    private boolean enabled;

    private Vision_Base vision;

    public SinglePipeline() {
        enabled = true;
    }

    @Override
    public void initializeVision(Mat mat) {
        vision.init(mat);
    }

    @Override
    protected void processInput(Mat input) {
        if (enabled) vision.processInput(input);
    }

    @Override
    protected void drawOutput(Mat output) {
        if (enabled) vision.drawOutput(output);
    }

    public final void enable() {
        enabled = true;
    }

    public final void disable() {
        enabled = false;
    }

    public final boolean getEnabled() {
        return enabled;
    }
}
