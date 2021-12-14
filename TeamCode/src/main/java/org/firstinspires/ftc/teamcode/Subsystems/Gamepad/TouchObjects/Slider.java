package org.firstinspires.ftc.teamcode.Subsystems.Gamepad.TouchObjects;

import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Touchpad;
import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Utils.Bounds;
import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Utils.Point;

//IMPORTANT PLEASE DONT SET VALUE TO 0 AUTOMATICALLY IN THIS CLASS THATS WHAT SNAPBACK IS FOR!

public class Slider extends ScaledTouchObject<Double> {

    protected final double finger;
    protected final Type s;
    protected Bounds b;
    protected boolean zoned = false;

    public Slider(Touchpad touchpad, Double defaultValue, int finger, double lowerOut, double upperOut, double exponent, Type s) {
        super(touchpad, defaultValue, -100, 100, lowerOut, upperOut, exponent);
        this.s = s;
        this.finger = finger;
    }

    public Slider(Touchpad touchpad, Double defaultValue, int finger, Type s) {
        super(touchpad, defaultValue, -100, 100);
        this.finger = finger;
        this.s = s;
    }

    public Slider(Touchpad touchpad, Double defaultValue, Bounds b, int finger, double lowerOut, double upperOut, double exponent, Type s) {
        super(touchpad, defaultValue, -100, 100, lowerOut, upperOut, exponent);
        this.s = s;
        this.finger = finger;
        this.b = b;
        zoned = true;
    }

    public Slider(Touchpad touchpad, Double defaultValue, int finger, Bounds b, Type s) {
        super(touchpad, defaultValue, -100, 100);
        this.finger = finger;
        this.s = s;
        this.b = b;
        zoned = true;
    }

    public Slider(Touchpad touchpad, Double defaultValue, Bounds b, Type s) {

        super(touchpad, defaultValue, -100, 100);
        this.s = s;
        this.finger = 0;
        this.b = b;
        zoned = true;

    }

    @Override
    public void updateInput() {

        Point fingerOne = new Point(touchpad.getFingerOneX(), touchpad.getFingerOneY());
        Point fingerTwo = new Point(touchpad.getFingerTwoX(), touchpad.getFingerTwoY());

        if (zoned) {
            switch(s) {
                case X_AXIS:
                    if(b.contains(fingerOne) || b.contains(fingerTwo)) {

                        if(finger == 0) {

                            if(b.contains(fingerOne) && !b.contains(fingerTwo)) {
                                value = scale(touchpad.getFingerOneX());
                            }
                            else if(b.contains(fingerTwo) && b.contains(fingerOne)) {
                                value = scale(touchpad.getFingerTwoX());
                            }

                        }
                        else if(finger == 1) {
                            value = scale(touchpad.getFingerOneX());
                        }
                        else {
                            value = scale(touchpad.getFingerTwoX());
                        }

                    }
                    break;
                case Y_AXIS:
                    if(b.contains(fingerOne) || b.contains(fingerTwo)) {

                        if(finger == 0) {

                            if(b.contains(fingerOne) && !b.contains(fingerTwo)) {
                                value = scale(touchpad.getFingerOneY());
                            }
                            else if(b.contains(fingerTwo) && b.contains(fingerOne)) {
                                value = scale(touchpad.getFingerTwoY());
                            }

                        }
                        else if(finger == 1) {
                            value = scale(touchpad.getFingerOneY());
                        }
                        else {
                            value = scale(touchpad.getFingerTwoY());
                        }

                    }
                    break;

                default:
                    value = 0d;

            }
        }
        else {
            switch(s) {
                case X_AXIS:
                    value = finger == 1 ? scale(touchpad.getFingerOneX()) : scale(touchpad.getFingerTwoX());
                    break;
                case Y_AXIS:
                    value = finger == 1 ? scale(touchpad.getFingerOneY()) : scale(touchpad.getFingerOneY());
                    break;
                default:
                    value = 0d;
            }
        }

    }
}
