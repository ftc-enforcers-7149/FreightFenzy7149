package org.firstinspires.ftc.teamcode.Subsystems.Sensors;

import android.util.Log;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Utils.Input;

public class FSR406 extends Sensor {

    public static final double VOLTAGE = 3.3 /*volts*/;
    public static final double MAX_FSR_RESISTANCE = 10 /*megaohms*/ * 1000000 /*conversion factor*/;
    public static final double ERROR_VALUE = Double.MAX_VALUE;
    private boolean excludeZero = false;

    private final double EXT_RESISTANCE;

    private AnalogInput fsr406;

    private double weight;
    private double lastNonOutlier;

    public FSR406(HardwareMap h, String sensorName, double EXT_RESISTANCE, int smoothing) {

        super(smoothing);
        fsr406 = h.analogInput.get(sensorName);
        this.EXT_RESISTANCE = EXT_RESISTANCE;

    }

    public FSR406(HardwareMap h, String sensorName, double EXT_RESISTANCE, int smoothing, boolean excludeZero) {

        super(smoothing);
        fsr406 = h.analogInput.get(sensorName);
        this.EXT_RESISTANCE = EXT_RESISTANCE;
        this.excludeZero = excludeZero;

    }

    public void updateInput() {

        // original equation is Voutput = (ext resistance * voltage) / (ext resistance + current sensor resistance)
        // rearranged to find the resistance of the sensor
        double currResistance = EXT_RESISTANCE * (VOLTAGE - fsr406.getVoltage()) / fsr406.getVoltage();

        if(!excludeZero || !(currResistance == 0)) {
            super.add(1000000 / currResistance);
            Log.i("Adding resistance of ", String.valueOf(currResistance));
        }
        else clearFilterVals();

        super.updateInput();

        weight = getValue();

    }

    public double getLastNonOutlier() { return super.lastNonOutlier; }

    public double getWeight() { return weight; }

    public void setExcludeZero(boolean excludeZero) {

        this.excludeZero = excludeZero;

    }

}
