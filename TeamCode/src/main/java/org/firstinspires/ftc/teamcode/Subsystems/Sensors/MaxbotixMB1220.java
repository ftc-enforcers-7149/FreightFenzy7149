package org.firstinspires.ftc.teamcode.Subsystems.Sensors;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Input;

public class MaxbotixMB1220 extends Sensor {

    public AnalogInput mb1220;
    private VOLTAGE v;
    private BulkRead bRead = null;
    private int portNum;

    private double distance;
    public static final double ERROR_VALUE = Double.MAX_VALUE;
    public static final double THREEV_POWER = 3.2 /*mV*/ / 1000;
    public static final double FIVEV_POWER = 4.9 /*mV*/ / 1000;

    public enum VOLTAGE {

        THREE,
        FIVE

    }

    public MaxbotixMB1220(HardwareMap h, String name, VOLTAGE v, int smoothing) {

        super(smoothing);
        mb1220 = h.analogInput.get(name);
        portNum = Integer.parseInt(mb1220.getConnectionInfo().split("; analog port ")[1]);
        this.v = v;

    }

    public MaxbotixMB1220(HardwareMap h, String name, BulkRead bRead, VOLTAGE v, int smoothing) {

        super(smoothing);
        mb1220 = h.analogInput.get(name);
        this.bRead = bRead;
        portNum = Integer.parseInt(mb1220.getConnectionInfo().split("; analog port ")[1]);
        this.v = v;

    }

    @Override
    public void updateInput() {

        try {

            super.add((bRead != null ? bRead.getAnalogValue(portNum) : mb1220.getVoltage())
                    / (v == VOLTAGE.THREE ? THREEV_POWER : FIVEV_POWER));
            super.updateInput();
            distance = super.getValue() * 2;

        }
        catch(Exception e) {

            distance = ERROR_VALUE;

        }

    }

    public VOLTAGE getVoltage() {return v;}
    public void setVoltage(VOLTAGE v) {this.v = v;}

    public double getDistance(DistanceUnit u) {

        switch(u) {

            case INCH:
                return distance * 0.393701d;
            case CM:
            default:
                return distance;

        }

    }
}
