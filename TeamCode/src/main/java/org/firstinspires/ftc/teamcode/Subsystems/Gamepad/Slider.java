package org.firstinspires.ftc.teamcode.Subsystems.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Utils.Scale;

public class Slider extends ScaledTouchObject<Double> {

    private double finger;
    Scale scale;
    SwipeType s;

    public Slider(String name, Touchpad touchpad, double exponent, double lowerOut, double upperOut, SwipeType s) {
        super(name, touchpad, exponent, lowerOut, upperOut);
        this.s = s;
    }

    public void setLowerIn() {
        setLowerIn(-100);
    }

    public void setUpperIn() {
        setUpperIn(100);
    }

    public Double update() {
        switch(s) {

            case HORIZ_AXIS:

                return finger == 1 ? scale.output(touchpad.getFingerOneX()) : scale.output(touchpad.getFingerTwoX());

            case VERT_AXIS:
                return finger == 1 ? scale.output(touchpad.getFingerOneY()) : scale.output(touchpad.getFingerOneY());

            default:
                return 0d;

        }

    }


}
