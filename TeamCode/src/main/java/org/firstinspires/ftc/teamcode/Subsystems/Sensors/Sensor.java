package org.firstinspires.ftc.teamcode.Subsystems.Sensors;

import org.firstinspires.ftc.teamcode.Subsystems.Utils.Input;

import java.util.ArrayList;
import java.util.Collections;

// sensor impl with smoothing for numerical vals.
public class Sensor implements Input {

    // generic type
    private ArrayList<Double> filterVals;
    private int smoothingSize;
    private boolean enableQuartileSmoothing = false;

    private double value;

    public Sensor(int smoothingSize) {

        this.smoothingSize = smoothingSize;
        filterVals = new ArrayList<>();

    }

    // uses simple MAA to smoothe out output. weee. todo maybe add in ways to toggle low/hipass
    @Override
    public void updateInput() {

        if(filterVals.size() < smoothingSize) {
            value = 0;
        }
        else {

            if(!enableQuartileSmoothing) {

                double sum = 0;
                for(Double t : filterVals) { sum += t; }
                value = sum / smoothingSize;

            }
            else {
                ArrayList<Double> dupli = filterVals;
                Collections.sort(dupli);

                double q1 = dupli.get((int) Math.ceil(dupli.size() / 4d));
                double q3 = dupli.get((int) Math.floor(3 * dupli.size() / 4d));
                double bound = 1.5d * (q3 - q1);

                double sum = 0;
                double iter = 0;

                for (double i : dupli) {

                    if (i >= q1 - bound && i <= q3 + bound) {

                        sum += i;
                        iter++;

                    }

                }

                value = sum / iter;

            }

        }

    }

    public void add(double newVal) {

        if(filterVals.size() + 1 > smoothingSize) filterVals.remove(0);
        filterVals.add(newVal);

    }

    public void setSmoothingSize(int smoothingSize) {
        this.smoothingSize = smoothingSize;
    }

    public double getValue() { return value; }

    public void setQuartileSmoothing(boolean enableQuartileSmoothing) { this.enableQuartileSmoothing = enableQuartileSmoothing;}

}
