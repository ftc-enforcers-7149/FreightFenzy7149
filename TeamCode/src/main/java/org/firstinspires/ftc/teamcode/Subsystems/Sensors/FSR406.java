package org.firstinspires.ftc.teamcode.Subsystems.Sensors;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Utils.Input;

public class FSR406 extends Sensor {

    public static final double VOLTAGE = 3.3 /*volts*/;
    public static final double MAX_FSR_RESISTANCE = 10 /*megaohms*/ * 1000000 /*conversion factor*/;
    public static final double ERROR_VALUE = Double.MAX_VALUE;

    private final double EXT_RESISTANCE;

    private AnalogInput fsr406;

    private double force;


    public FSR406(HardwareMap h, String sensorName, double EXT_RESISTANCE, int smoothing) {

        super(smoothing);
        fsr406 = h.analogInput.get(sensorName);
        this.EXT_RESISTANCE = EXT_RESISTANCE;

    }

    public void updateInput() {

        // original equation is Voutput = (ext resistance * voltage) / (ext resistance + current sensor resistance)
        // rearranged to find the resistance of the sensor
        double currResistance = EXT_RESISTANCE * (VOLTAGE - fsr406.getVoltage()) / fsr406.getVoltage();

        super.add(currResistance);
        super.updateInput();

        force = getValue();

    }

    public double getForce() { return force; }

}
