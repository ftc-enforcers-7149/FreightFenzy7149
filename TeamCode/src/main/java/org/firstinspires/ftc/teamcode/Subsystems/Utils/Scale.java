package org.firstinspires.ftc.teamcode.Subsystems.Utils;

// TODO: maybe make this an abstract function in a parent class where lower and upper in are declared per child class?
public class Scale {

    private double exponent, lowerIn = -100, upperIn = 100, lowerOut, upperOut;

    public Scale(double lowerOut, double upperOut, double exponent) {

        this.lowerOut = lowerOut;
        this.upperOut = upperOut;
        this.exponent = exponent;

    }

    public double output(double input) {
        double curve = Math.signum(input) * Math.abs(Math.pow(input, exponent));
        double scaleFactor = ((upperOut - lowerOut) / (upperIn - lowerIn));
        double scale = scaleFactor * curve + (upperOut - scaleFactor*upperIn);

        return scale;
    }

}