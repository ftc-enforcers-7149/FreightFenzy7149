package org.firstinspires.ftc.teamcode.Subsystems.Sensors;

import org.firstinspires.ftc.teamcode.Subsystems.Utils.Input;

import java.util.ArrayList;
import java.util.Collections;

// sensor impl with smoothing for numerical vals.
public class Sensor implements Input {

    // generic type
    protected ArrayList<Double> filterVals;
    protected ArrayList<Double> outliers;
    protected int smoothingSize;
    protected boolean enableQuartileSmoothing = false, enableHighPass = false, enableLowPass = false;
    protected double highPassMax = 0, lowPassMax = 0;

    protected double value;

    public Sensor(int smoothingSize) {

        this.smoothingSize = smoothingSize;
        filterVals = new ArrayList<>();
        outliers = new ArrayList<>();

    }

    // uses simple MAA to smoothe out output. weee. todo maybe add in ways to toggle low/hipass
    @Override
    public void updateInput() {

        if(filterVals.size() < smoothingSize) {
            value = 0;
        }
        else {

            double sum = 0;
            for(Double t : filterVals) { sum += t; }
            value = sum / smoothingSize;

        }

    }

    public void add(double newVal) {

        if(enableHighPass && newVal > highPassMax || enableLowPass && newVal < lowPassMax) return;

        if(filterVals.size() < smoothingSize) {

            filterVals.add(newVal);
            return;

        }

        if(enableQuartileSmoothing) {

            ArrayList<Double> dupli = (ArrayList<Double>) filterVals.clone();

            double q1 = dupli.get((int) Math.ceil(dupli.size() / 4d));
            double q3 = dupli.get((int) Math.floor(3 * dupli.size() / 4d));
            double bound = 1.5d * (q3 - q1);

            if(newVal > q3 + bound || newVal < q1 - bound) {

                outliers.add(newVal);

                if(outliers.size() > smoothingSize) {

                    outliers.remove(0);

                    dupli = (ArrayList<Double>) outliers.clone();
                    q1 = dupli.get((int) Math.ceil(dupli.size() / 4d));
                    q3 = dupli.get((int) Math.floor(3 * dupli.size() / 4d));
                    bound = 1.5d * (q3 - q1);

                    int fails = 0;

                    for(double i : dupli) {

                        if(i > q3 + bound || i < q1 - bound) fails++;

                    }

                    if(fails / (double) dupli.size() < .5) filterVals = (ArrayList<Double>) outliers.clone();

                }

            }
            else {

                if(filterVals.size() + 1 > smoothingSize) filterVals.remove(0);
                filterVals.add(newVal);

            }

        }
        else {

            if(filterVals.size() + 1 > smoothingSize) filterVals.remove(0);
            filterVals.add(newVal);

        }

    }

    public void setSmoothingSize(int smoothingSize) {
        this.smoothingSize = smoothingSize;
        filterVals.clear();
        outliers.clear();
    }

    public void setHighPass(boolean enableHighPass) {

        this.enableHighPass = enableHighPass;
        this.highPassMax = 420d;

    }

    public void setLowPass(boolean enableLowPass) {

        this.enableLowPass = enableLowPass;
        this.lowPassMax = 69d;

    }

    public void setHighPass(boolean enableHighPass, double highPassMax) {

        this.enableHighPass = enableHighPass;
        this.highPassMax = highPassMax;

    }

    public void setLowPass(boolean enableLowPass, double lowPassMax) {

        this.enableLowPass = enableLowPass;
        this.lowPassMax = lowPassMax;

    }

    public double getValue() { return value; }

    public void setQuartileSmoothing(boolean enableQuartileSmoothing) { this.enableQuartileSmoothing = enableQuartileSmoothing;}

}
