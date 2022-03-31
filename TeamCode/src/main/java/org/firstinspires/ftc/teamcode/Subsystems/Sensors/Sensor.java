package org.firstinspires.ftc.teamcode.Subsystems.Sensors;

import org.firstinspires.ftc.teamcode.Subsystems.Utils.Input;

import java.util.ArrayList;

// sensor impl with smoothing for numerical vals.
public class Sensor implements Input {

    // generic type
    private ArrayList<Double> filterVals;
    private int smoothingSize;

    private double value;

    public Sensor(int smoothingSize) {

        this.smoothingSize = smoothingSize;
        filterVals = new ArrayList<>();

    }

    // uses simple MAA to smoothe out output. weee. todo maybe add in ways to toggle low/hipass
    @Override
    public void updateInput() {

        if(filterVals.size() < smoothingSize) {
            value = 0; // i know the ide doesnt like this but oh well
        }
        else {
            double sum = 0;
            for(Double t : filterVals) { sum += t; }
            value = sum / smoothingSize;
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

}
