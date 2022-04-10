package org.firstinspires.ftc.teamcode.Subsystems.Sensors;

import org.firstinspires.ftc.teamcode.Subsystems.Utils.Input;

public abstract class FreightDetector implements Input {

    protected FreightType currentType;
    protected double currentWeight, initialWeight;
    protected boolean lookForAverage;
    protected boolean freightInIntake, lastFreightInIntake;

    public final double WEIGHT_TOLERANCE;

    public enum FreightType {

        NONE(0),
        DUCK(20),
        BALL_BLOCK(26),
        MEDIUM_BLOCK(35),
        HEAVY_BLOCK(53);

        private final double weight;

        private FreightType(double weight) {

            this.weight = weight;

        }

        public double getWeight() {

            return weight;

        }

    }

    public FreightDetector(double WEIGHT_TOLERANCE) {

        this.WEIGHT_TOLERANCE = WEIGHT_TOLERANCE;

    }

    @Override
    public void updateInput() {

        if (currentWeight < FreightType.DUCK.weight) {
            currentType = FreightType.NONE;
            freightInIntake = false;
            //lookForAverage = false;
        }
        else
            freightInIntake = true;

//        if (!freightInIntake && lookForAverage) {
//            if (getLastNonOutlier() >= initialWeight) {
//                initialWeight = currentWeight;
//                freightInIntake = true;
//                lookForAverage = false;
//            }
//            //else
//            //    lookForAverage = false;
//        }

        if (freightInIntake && !lastFreightInIntake) {
            initialWeight = currentWeight;
            if (currentWeight < FreightType.BALL_BLOCK.weight)
                currentType = FreightType.DUCK;
            else if (currentWeight < FreightType.MEDIUM_BLOCK.weight)
                currentType = FreightType.BALL_BLOCK;
            else if (currentWeight < FreightType.HEAVY_BLOCK.weight)
                currentType = FreightType.MEDIUM_BLOCK;
            else
                currentType = FreightType.HEAVY_BLOCK;
        }
        else if (!freightInIntake && lastFreightInIntake) {
            //lookForAverage = false;
            //initialWeight = 0;
        }

//        if(currentWeight < FreightType.DUCK.weight + WEIGHT_TOLERANCE && currentWeight > 0.5)
//            currentType = FreightType.DUCK;
//        else if(currentWeight < FreightType.BALL_BLOCK.weight + WEIGHT_TOLERANCE && currentWeight > FreightType.BALL_BLOCK.weight - WEIGHT_TOLERANCE)
//            currentType = FreightType.BALL_BLOCK;
//        else if(currentWeight < FreightType.MEDIUM_BLOCK.weight + WEIGHT_TOLERANCE && currentWeight > FreightType.MEDIUM_BLOCK.weight - WEIGHT_TOLERANCE)
//            currentType = FreightType.MEDIUM_BLOCK;
//        else if(currentWeight < FreightType.HEAVY_BLOCK.weight + WEIGHT_TOLERANCE && currentWeight > FreightType.HEAVY_BLOCK.weight - WEIGHT_TOLERANCE)
//            currentType = FreightType.HEAVY_BLOCK;
//        else {
//            currentType = FreightType.NONE;
//        }


//        if(currentWeight > 0 && currentWeight < 1e7) {
//            if(!freightInIntake) initialWeight = currentWeight;
//            freightInIntake = true;
//        }
//        else {
//
//            initialWeight = 0;
//            freightInIntake = false;
//
//        }

        lastFreightInIntake = freightInIntake;
    }

    public abstract void updateWeight();
    public abstract double getLastNonOutlier();

    public FreightType getCurrentType() {

        return currentType;

    }

    public boolean isFreightInIntake() {return freightInIntake;}

    public double getCurrentWeight() {
        return currentWeight;
    }

    public double getInitialWeight() {
        return initialWeight;
    }

}
