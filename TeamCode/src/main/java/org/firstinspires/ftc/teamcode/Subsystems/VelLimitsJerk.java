package org.firstinspires.ftc.teamcode.Subsystems;

/**
 * A curve, derived using Calculus, that limits jerk to a constant while
 * treating the output as a velocity. The input is treated as a desired velocity,
 * which will be reached in some time based on this curve:
 * https://www.desmos.com/calculator/lfzqplkkgg
 */
public class VelLimitsJerk {

    //Used to determine maximum jerk constant
    private double maxValue;
    private double maxTime;

    //Scales input such that it stays within maxValue
    private double inputRatio;

    //Where and when the curve "starts"
    private double startValue;
    private double startTime;

    //Used to determine when the curve should be redetermined
    private double lastInput;

    /**
     * @param maxValue The maximum output value
     * @param maxInput The maximum input value
     * @param maxTime The desired time to reach the maximum output value starting at 0
     */
    public VelLimitsJerk(double maxValue, double maxInput, double maxTime) {
        this.maxValue = maxValue;
        this.maxTime = maxTime;
        inputRatio = maxValue / maxInput;

        lastInput = 0;
    }

    /**
     * Updates the output from the velocity curve based on the input and time
     * @param input Input / Desired output
     * @return Output from curve
     */
    public double update(double input) {
        double currTime = System.currentTimeMillis();

        input *= inputRatio;

        if (input != lastInput) {
            startValue = velFunction(Math.abs(lastInput), startValue, currTime - startTime);
            startTime = currTime;
        }

        double velocity = velFunction(input, startValue, currTime - startTime);

        lastInput = input;
        return velocity;
    }

    /**
     * https://www.desmos.com/calculator/lfzqplkkgg
     * This velocity curve is based on the 2nd anti-derivative of a constant jerk function.
     * The jerk is determined from what is needed to reach maxValue in maxTime from 0.
     * @param value Input / Desired output value
     * @param startValue What value the curve should start at
     * @param time The time (in ms) since the curve started
     * @return The curve's output
     */
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

    /**
     * @param maxValue The maximum output value
    */
    public void setMaxValue(double maxValue) {
        this.maxValue = maxValue;
    }

    /**
    * @param maxInput The maximum input value
    */
    public void setMaxInput(double maxInput) {
        inputRatio = maxValue / maxInput;
    }

    /**
     * @param maxTime The desired time to reach the maximum output value starting at 0
     */
    public void setMaxTime(double maxTime) {
        this.maxTime = maxTime;
    }
}
