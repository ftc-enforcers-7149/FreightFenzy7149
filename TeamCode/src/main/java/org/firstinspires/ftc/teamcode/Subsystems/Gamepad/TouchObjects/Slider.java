package org.firstinspires.ftc.teamcode.Subsystems.Gamepad.TouchObjects;

import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Touchpad;
import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Utils.Bounds;
import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Utils.Point;
import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Utils.Scale;

//IMPORTANT PLEASE DONT SET VALUE TO 0 AUTOMATICALLY IN THIS CLASS THATS WHAT SNAPBACK IS FOR!
public class Slider extends ScaledTouchObject<Double> {

    public enum SliderType {
        X_AXIS,
        Y_AXIS
    }

    protected final SliderType sliderType;
    protected final Bounds bounds;

    public Slider(Touchpad touchpad, Double defaultValue, SliderType sliderType, Bounds bounds,
                  double lowerOut, double upperOut) {
        super(touchpad, defaultValue, new Scale(
                sliderType == SliderType.X_AXIS ? bounds.getLeftX() : bounds.getBottomY(),
                sliderType == SliderType.X_AXIS ? bounds.getRightX() : bounds.getTopY(),
                lowerOut, upperOut
        ));
        this.sliderType = sliderType;
        this.bounds = bounds;
    }

    public Slider(Touchpad touchpad, Double defaultValue, SliderType sliderType,
                  double lowerOut, double upperOut) {
        this(touchpad, defaultValue, sliderType,
                new Bounds(-100, 100, -100, 100), lowerOut, upperOut);
    }

    @Override
    public void updateInput() {
        Point fingerOne = touchpad.getFingerOne();
        Point fingerTwo = touchpad.getFingerTwo();

        switch(sliderType) {
            case X_AXIS:
                if (touchpad.getNumFingers() == 2 && bounds.contains(fingerTwo))
                    value = scale(fingerTwo.getX());
                else if (touchpad.getNumFingers() == 1 && bounds.contains(fingerOne))
                    value = scale(fingerOne.getX());
                return;
            case Y_AXIS:
                if (bounds.contains(fingerTwo))
                    value = scale(fingerTwo.getY());
                else if (bounds.contains(fingerOne))
                    value = scale(fingerOne.getY());
                return;
            default:
                value = 0d;
        }
    }
}
