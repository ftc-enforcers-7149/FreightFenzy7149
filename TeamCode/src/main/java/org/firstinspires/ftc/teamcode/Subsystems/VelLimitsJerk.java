package org.firstinspires.ftc.teamcode.Subsystems;

public class VelLimitsJerk {

    private double maxValue;
    private double maxInput;
    private double maxTime;

    private double startValue;
    private double startTime;

    private double lastInput;

    public VelLimitsJerk(double maxValue, double maxInput, double maxTime) {
        this.maxValue = maxValue;
        this.maxInput = maxInput;
        this.maxTime = maxTime;

        lastInput = 0;
    }

    public double update(double input) {
        double currTime = System.currentTimeMillis();

        input *= maxValue / maxInput;

        if (input != lastInput) {
            startValue = velFunction(Math.abs(lastInput), startValue, currTime - startTime);
            startTime = currTime;
        }

        double velocity = velFunction(input, startValue, currTime - startTime);

        lastInput = input;
        return velocity;
    }

    private double velFunction(double value, double startValue, double time) {
        double sign = Math.signum(value - startValue);
        double jerk = sign * 4*maxValue/(maxTime*maxTime);
        double timeHalf = Math.sqrt((value - startValue)/jerk);

        if (time <= timeHalf) {
            return jerk * (0.5 * (time*time))
                    + startValue;
        }
        else if (time < 2 * timeHalf) {
            return jerk * (-0.5 * (time*time) + timeHalf * (2*time - timeHalf))
                    + startValue;
        }
        else {
            return value;
        }
    }

    public void setMaxValue(double maxValue) {
        this.maxValue = maxValue;
    }

    public void setMaxInput(double maxInput) {
        this.maxInput = maxInput;
    }

    public void setMaxTime(double maxTime) {
        this.maxTime = maxTime;
    }
}
