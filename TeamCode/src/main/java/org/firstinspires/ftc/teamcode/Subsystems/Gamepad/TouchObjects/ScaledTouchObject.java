package org.firstinspires.ftc.teamcode.Subsystems.Gamepad.TouchObjects;

import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Touchpad;

public abstract class ScaledTouchObject<T> extends TouchObject<T> {

    private final double lowerOut, upperOut, exponent, lowerIn, upperIn;

    public ScaledTouchObject(Touchpad touchpad, T defaultValue, double lowerIn, double upperIn, double lowerOut, double upperOut, double exponent) {
        super(touchpad, defaultValue);
        this.lowerIn = lowerIn; this.upperIn = upperIn;
        this.lowerOut = lowerOut; this.upperOut = upperOut;
        this.exponent = exponent;
    }

    public ScaledTouchObject(Touchpad touchpad, T defaultValue, double lowerIn, double upperIn) {
        this(touchpad, defaultValue, lowerIn, upperIn, 0, 1, 1);
    }

    public double scale(double input) {
        double curve = Math.signum(input) * Math.abs(Math.pow(input, exponent));
        double scaleFactor = ((upperOut - lowerOut) / (upperIn - lowerIn));
        double scale = scaleFactor * curve + (upperOut - scaleFactor*upperIn);

        return scale;
    }
}
