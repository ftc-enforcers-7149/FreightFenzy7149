package org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Utils;

// TODO: maybe make this an abstract function in a parent class where lower and upper in are declared per child class?
public class Scale {

    private final double lowerIn, upperIn, lowerOut, upperOut;

    public Scale(double lowerIn, double upperIn, double lowerOut, double upperOut) {
        this.lowerIn = lowerIn;
        this.upperIn = upperIn;
        this.lowerOut = lowerOut;
        this.upperOut = upperOut;
    }

    public double output(double input) {
        double scaleFactor = (upperOut - lowerOut) / (upperIn - lowerIn);
        double scale = upperOut - scaleFactor*upperIn;

        return scale;
    }
}