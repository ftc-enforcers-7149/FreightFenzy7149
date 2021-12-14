package org.firstinspires.ftc.teamcode.Subsystems.Gamepad.TouchObjects;

import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Touchpad;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Scale;

public class Slider extends ScaledTouchObject<Double> {

    double finger;
    Scale scale;
    Type s;
    Zone z;
    boolean zoned = false;

    public Slider(String name, Touchpad touchpad, int finger, double exponent, double lowerOut, double upperOut, Type s) {
        super(name, touchpad, exponent, lowerOut, upperOut);
        this.s = s;
        this.finger = finger;
    }

    public Slider(String name, Touchpad touchpad, int finger, Type s) {

        super(name, touchpad);
        this.finger = finger;
        this.s = s;

    }

    public Slider(String name, Touchpad touchpad, Zone z, int finger, double exponent, double lowerOut, double upperOut, Type s) {
        super(name, touchpad, exponent, lowerOut, upperOut);
        this.s = s;
        this.finger = finger;
        this.z = z;
        zoned = true;
    }

    public Slider(String name, Touchpad touchpad, int finger, Zone z, Type s) {

        super(name, touchpad);
        this.finger = finger;
        this.s = s;
        this.z = z;

    }

    public void setLowerIn() {
        setLowerIn(-100);
    }

    public void setUpperIn() {
        setUpperIn(100);
    }

    public Double update() {

        if (zoned) {
            switch(s) {

                case X_AXIS:

                    if(z.update()) return finger == 1 ? scale.output(touchpad.getFingerOneX()) : scale.output(touchpad.getFingerTwoX());

                case Y_AXIS:
                    if(z.update()) return finger == 1 ? scale.output(touchpad.getFingerOneY()) : scale.output(touchpad.getFingerOneY());

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


}
