package org.firstinspires.ftc.teamcode.Subsystems.Gamepad.TouchObjects;

import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Touchpad;

public abstract class ScaledTouchObject<T> extends TouchObject {

    private double lowerOut, upperOut;
    private double lowerIn, upperIn;
    private double exponent;

    public ScaledTouchObject(String name, Touchpad touchpad, double exponent, double lowerOut, double upperOut) {
        super(name, touchpad);
        this.lowerOut = lowerOut; this.upperOut = upperOut;
        this.exponent = exponent;
    }

    public ScaledTouchObject(String name, Touchpad touchpad) {

        super(name, touchpad);
        this.lowerOut = 0; this.upperOut = 1;
        this.exponent = 1;

    }

    public double scale(double input) {
        double curve = Math.signum(input) * Math.abs(Math.pow(input, exponent));
        double scaleFactor = ((upperOut - lowerOut) / (upperIn - lowerIn));
        double scale = scaleFactor * curve + (upperOut - scaleFactor*upperIn);

        return scale;
    }

    public abstract void setLowerIn();
    public abstract void setUpperIn();

    public void setLowerIn(double lowerIn) { this.lowerIn = lowerIn; }
    public void setUpperIn(double upperIn) { this.upperIn = upperIn; }

}
