package org.firstinspires.ftc.teamcode.Matttu.VisionStructure;

import android.util.ArrayMap;

import org.opencv.core.Mat;

import java.util.function.BiConsumer;

public class MultiPipeline extends Pipeline_Base {

    private ArrayMap<Vision_Base, Boolean> visions;

    public MultiPipeline(Vision_Base... visions) {
        this.visions = new ArrayMap<Vision_Base, Boolean>();

        for (Vision_Base vision : visions) {
            this.visions.put(vision, true);
        }
    }

    @Override
    public void initializeVision(Mat mat) {
        visions.forEach(new BiConsumer<Vision_Base, Boolean>() {
            @Override
            public void accept(Vision_Base vision, Boolean enabled) {
                vision.init(mat);
            }
        });
    }

    @Override
    protected void processInput(Mat input) {
        visions.forEach(new BiConsumer<Vision_Base, Boolean>() {
            @Override
            public void accept(Vision_Base vision, Boolean enabled) {
                if (enabled) vision.processInput(input);
            }
        });
    }

    @Override
    protected void drawOutput(Mat output) {
        visions.forEach(new BiConsumer<Vision_Base, Boolean>() {
            @Override
            public void accept(Vision_Base vision, Boolean enabled) {
                if (enabled) vision.drawOutput(output);
            }
        });
    }

    public void addPipeline(Vision_Base... visions) {
        for (Vision_Base vision : visions) {
            this.visions.put(vision, true);
        }
    }

    public void removePipeline(Vision_Base vision) {
        visions.remove(vision);
    }
}
