package org.firstinspires.ftc.teamcode.Subsystems.Sensors;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class WeightFreightDetector extends FreightDetector {

    public FSR406 fsr406;

    public WeightFreightDetector(HardwareMap h, String sensorName, double EXT_RESISTANCE, int smoothing) {

        super(5);
        fsr406 = new FSR406(h, sensorName, EXT_RESISTANCE, smoothing);
        fsr406.setQuartileSmoothing(true);
    }

    public WeightFreightDetector(HardwareMap h, String sensorName, int smoothing) {

        super(5);
        fsr406 = new FSR406(h, sensorName, 10000, smoothing);
        fsr406.setQuartileSmoothing(true);
    }

    public WeightFreightDetector(HardwareMap h, String sensorName, double EXT_RESISTANCE, int smoothing, double WEIGHT_TOLERANCE) {

        super(WEIGHT_TOLERANCE);
        fsr406 = new FSR406(h, sensorName, EXT_RESISTANCE, smoothing);
        fsr406.setQuartileSmoothing(true);
    }

    public WeightFreightDetector(HardwareMap h, String sensorName, int smoothing, double WEIGHT_TOLERANCE) {

        super(WEIGHT_TOLERANCE);
        fsr406 = new FSR406(h, sensorName, 10000, smoothing);
        fsr406.setQuartileSmoothing(true);
    }

    @Override
    public void updateWeight() {

        super.currentWeight = fsr406.getWeight();

    }

    @Override
    public void updateInput() {

        updateWeight();
        fsr406.updateInput();
        super.updateInput();

    }

    @Override
    public double getLastNonOutlier() {
        return fsr406.getLastNonOutlier();
    }
}
