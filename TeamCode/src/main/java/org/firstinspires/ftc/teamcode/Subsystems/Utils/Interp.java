package org.firstinspires.ftc.teamcode.Subsystems.Utils;

public class Interp {
    private final double startPos, endPos, interpTime, startTime;
    private double deltaTime;

    public enum Method {
        INSTANT, LINEAR, EXPONENTIAL, SMOOTH_ACCEL
    }
    private final Method method;
    private double scaleFactor;

    public Interp(double startPos, double endPos, double interpTime, Method method) {
        this.startPos = startPos;
        this.endPos = endPos;
        this.interpTime = interpTime;
        this.startTime = System.currentTimeMillis();
        this.deltaTime = 0;

        this.method = method;
        this.scaleFactor = 0;
    }

    public Interp(double startPos, double endPos, double interpTime, Method method, double scaleFactor) {
        this(startPos, endPos, interpTime, method);
        this.scaleFactor = scaleFactor;
    }

    public void setScaleFactor(double scaleFactor) {
        this.scaleFactor = scaleFactor;
    }

    public double getValue() {
        deltaTime = System.currentTimeMillis() - startTime;

        if (deltaTime > interpTime) return endPos;

        switch (method) {
            case LINEAR:
                return ((deltaTime / interpTime) * (endPos - startPos)) + startPos;
            case EXPONENTIAL:
                return (Math.pow(deltaTime / interpTime, scaleFactor) * (endPos - startPos)) + startPos;
            case SMOOTH_ACCEL:
                double c = ((endPos - startPos) / 2) / Math.pow(interpTime / 2, scaleFactor);
                if (deltaTime <= interpTime / 2)
                    return (c * Math.pow(deltaTime, scaleFactor)) + startPos;
                else
                    return (-c * Math.pow(interpTime - deltaTime, scaleFactor)) + startPos;
            case INSTANT:
            default:
                return endPos;
        }
    }
}
