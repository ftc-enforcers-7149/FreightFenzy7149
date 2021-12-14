package org.firstinspires.ftc.teamcode.Subsystems.Gamepad.TouchObjects;

import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Touchpad;

public class Snapback extends Slider {

    public Snapback(String name, Touchpad touchpad, int finger, double exponent, double lowerOut, double upperOut, Type s) {
        super(name, touchpad, finger, exponent, lowerOut, upperOut, s);
    }

    public Snapback(String name, Touchpad touchpad, int finger, Type s) {
        super(name, touchpad, finger, s);
    }

    public Snapback(String name, Touchpad touchpad, Zone z, int finger, double exponent, double lowerOut, double upperOut, Type s) {
        super(name, touchpad, z, finger, exponent, lowerOut, upperOut, s);
    }

    public Snapback(String name, Touchpad touchpad, int finger, Zone z, Type s) {
        super(name, touchpad, finger, z, s);
    }

    @Override
    public Double get() {

        if(finger == 1 ? touchpad.getNumFingers() >= 1 : touchpad.getNumFingers() == 2) {

            if (zoned) {
                switch(s) {

                    case X_AXIS:

                        if(z.get()) return finger == 1 ? scale.output(touchpad.getFingerOneX()) : scale.output(touchpad.getFingerTwoX());

                    case Y_AXIS:
                        if(z.get()) return finger == 1 ? scale.output(touchpad.getFingerOneY()) : scale.output(touchpad.getFingerOneY());

                    default:
                        return 0d;

                }
            }
            else {
                switch(s) {

                    case X_AXIS:

                        return finger == 1 ? scale.output(touchpad.getFingerOneX()) : scale.output(touchpad.getFingerTwoX());

                    case Y_AXIS:
                        return finger == 1 ? scale.output(touchpad.getFingerOneY()) : scale.output(touchpad.getFingerOneY());

                    default:
                        return 0d;

                }
            }

        }
        else {

            return 0d;

        }

    }
}
