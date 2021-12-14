package org.firstinspires.ftc.teamcode.Subsystems.Gamepad.TouchObjects;

import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Touchpad;

//TODO: fix this jank ass implementation lolzies
public class Snapback extends Slider {

    public Snapback(Touchpad touchpad, Double defaultValue, int finger, double lowerOut, double upperOut, double exponent, Type s) {
        super(touchpad, defaultValue, finger, lowerOut, upperOut, exponent, s);
    }

    public Snapback(Touchpad touchpad, Double defaultValue, int finger, Type s) {
        super(touchpad, defaultValue, finger, s);
    }

    public Snapback(Touchpad touchpad, Double defaultValue, Button z, int finger, double lowerOut, double upperOut, double exponent, Type s) {
        super(touchpad, defaultValue, z, finger, lowerOut, upperOut, exponent, s);
    }

    public Snapback(Touchpad touchpad, Double defaultValue, int finger, Button z, Type s) {
        super(touchpad, defaultValue, finger, z, s);
    }

    @Override
    public void updateInput() {
        if(finger == 1 ? touchpad.getNumFingers() >= 1 : touchpad.getNumFingers() == 2) {
            if (zoned) {
                switch(s) {
                    case X_AXIS:
                        if(z.get()) value = (finger == 1 ? scale(touchpad.getFingerOneX()) : scale(touchpad.getFingerTwoX()));
                        return;
                    case Y_AXIS:
                        if(z.get()) value = (finger == 1 ? scale(touchpad.getFingerOneY()) : scale(touchpad.getFingerOneY()));
                        return;
                    default:
                        value = 0d;
                }
            }
            else {
                switch(s) {
                    case X_AXIS:
                        value = (finger == 1 ? scale(touchpad.getFingerOneX()) : scale(touchpad.getFingerTwoX()));
                        return;
                    case Y_AXIS:
                        value = (finger == 1 ? scale(touchpad.getFingerOneY()) : scale(touchpad.getFingerOneY()));
                        return;
                    default:
                        value = 0d;
                }
            }
        }
        else {
            value = 0d;
        }
    }
}
