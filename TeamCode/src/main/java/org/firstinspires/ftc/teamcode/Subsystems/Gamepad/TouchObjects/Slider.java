package org.firstinspires.ftc.teamcode.Subsystems.Gamepad.TouchObjects;

import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Touchpad;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Scale;

public class Slider extends ScaledTouchObject<Double> {

    protected final double finger;
    protected final Type s;
    protected Zone z;
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

    public Slider(Touchpad touchpad, Double defaultValue, Zone z, int finger, double lowerOut, double upperOut, double exponent, Type s) {
        super(touchpad, defaultValue, -100, 100, lowerOut, upperOut, exponent);
        this.s = s;
        this.finger = finger;
        this.z = z;
        zoned = true;
    }

    public Slider(Touchpad touchpad, Double defaultValue, int finger, Zone z, Type s) {
        super(touchpad, defaultValue, -100, 100);
        this.finger = finger;
        this.s = s;
        this.z = z;
        zoned = true;
    }

    @Override
    public void updateInput() {
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
                    value = finger == 1 ? scale(touchpad.getFingerOneX()) : scale(touchpad.getFingerTwoX());
                    return;
                case Y_AXIS:
                    value = finger == 1 ? scale(touchpad.getFingerOneY()) : scale(touchpad.getFingerOneY());
                    return;
                default:
                    value = 0d;
            }
        }
    }
}
