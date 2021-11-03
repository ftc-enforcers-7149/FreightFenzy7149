package org.firstinspires.ftc.teamcode.NovaLmao;

import java.util.concurrent.TimeUnit;

// https://www.desmos.com/calculator/qwbiczq3la
public class CurveHandling {

    private double projVelocity;

    // Normalization values
    private double maxValue, maxInput;
    private double inputRatio;

    //Implement later. For now, everything is normalized as if input is in milliseconds. Lol
    /*private TimeUnit timeUnit = TimeUnit.MILLISECONDS;*/

    // Rewrite this comment
    private double lastInput = 0, lastOutput = 0, lastVelocity = 0, lastTime = 0;
    private double startTime = 0;

    private CurveProfile c;
    private boolean disregardRev;

    public CurveHandling(double time, double maxValue, double maxInput, double revZone, double revTime, double maxTime) {

        this.startTime = time;
        this.maxValue = maxValue; this.maxInput = maxInput;
        c = new CurveProfile(revTime, maxTime, revZone, 0.1);

        inputRatio = maxValue / maxInput;
        /*timeRatio = 1 / timeUnit.toMillis(1);*/

    }

    public CurveHandling(double time, double maxValue, double maxInput, double revZone, double revTime, double maxTime, TimeUnit timeUnit) {

        this.startTime = time;
        this.maxValue = maxValue; this.maxInput = maxInput;
        c = new CurveProfile(revTime, maxTime, revZone, 0.1);
        /*this.timeUnit = timeUnit;*/

        inputRatio = maxValue / maxInput;

    }

    public CurveHandling(double time, double revZone, double revTime, double maxTime) {

        this.startTime = time;
        this.maxValue = 1; this.maxInput = 1;
        c = new CurveProfile(revTime, maxTime, revZone, 0.1);

        inputRatio = maxValue / maxInput;

    }

    //TODO: holy fucking shit proofread this dawg
    private double update(double time, double input) {

        // We normalize our input and set it to our output variable.
        double output = input * inputRatio;

        // If the output changes, we do this funky jazz

        if(output != lastOutput) {

            // Evaluates if the input has changed.
            boolean inputChange = lastInput != input;

            // Evaluates if we're in the controller deadzone.
            boolean isDeadzone = Math.abs(input - lastInput) <= c.deadzone;

            // Evaluates if we're within the rev input range.
            boolean isRev = c.deadzone > Math.abs(output);

            // Evaluates if we're decelerating.
            boolean isDecelerating = output < 0;

            // If our input has changed and we're outside the deadzone, update curve profile and start time.
            // If not, we don't change our profile.
            if (inputChange && !isDeadzone) {

                //Update curve profile.
                //TODO: I know this logic will probably be messed up with positive-negative change but
                // I can't be bothered to change it right now. Oh well
                c.update(Math.max(lastOutput, c.deadzone), output);
                disregardRev = c.revZone > Math.abs(output);

                //Update start time.
                startTime = time;

            }

            // Evaluates if we're within our maximum allotted profile time.
            boolean isTime = time <= c.maxTime;

            // If we're within our time profile and not in the deadzone, follow curve
            if (isTime && (!isDeadzone || !isDecelerating)) {

                // Sets output value based on appropriate curve.

                // If we're within our rev value and we need to use the curve, use rev curve:
                if (isRev && !disregardRev) {

                    // Calculates our slope modifier.
                    // The equation is a = (rev power - initial input power) / normalized rev time ^ 2
                    double a = (c.cRev - c.iOutput) / Math.pow(c.revTime / 1000, 2);

                    // Our overall equation
                    output = a * Math.pow(time - startTime, 2) + c.iOutput;

                }
                // If we're outside of our rev value, use max curve:
                else {

                    // Calculates our slope modifier.
                    // The equation is a = (rev power - desired input power) / (normalized rev time - normalized max time) ^ 2
                    double a = (c.cRev - c.dOutput) / Math.pow((c.revTime / 1000) - (c.maxTime / 1000), 2);

                    // Our overall equation
                    output = a * Math.pow((time - startTime) - (c.maxTime / 1000), 2) + c.dOutput;

                }

            }
        }

        //TODO: CHECK VALIDITY OF THIS COMMENT
        // Conditions for the output remaining the same include:
        // - we're outside our allotted profile time
        // - we've decelerated by a minimal value
        // - the output is the exact same

        lastInput = input;
        lastOutput = output;
        this.projVelocity = output;
        return output;

    }

    //TODO: confirm this runs the input function
    public boolean run (double time, double input) {

        return input == update(time, input);

    }

    public double getProjVelocity() {return projVelocity;}

    // Just for readability.

    public class CurveProfile {

        // General curve profile parameters. revTime and maxTime are currently in millis, revZone is the max output to
        // use on the rev curve, and deadzone is the input deadzone where nothing changes on acceleration, and where
        // deceleration automatically applies on decel.

        protected final double revTime, maxTime, revZone, deadzone;
        protected double iOutput, dOutput, cRev;

        public CurveProfile(double revTime, double maxTime, double revZone, double deadzone) {

            this.revTime = revTime; this.maxTime = maxTime; this.revZone = revZone; this.deadzone = deadzone;

        }

        public void update(double iOutput, double dOutput) {

            this.iOutput = iOutput; this.dOutput = dOutput; this.cRev = Math.copySign(revZone, dOutput - iOutput);

        }

    }

}
