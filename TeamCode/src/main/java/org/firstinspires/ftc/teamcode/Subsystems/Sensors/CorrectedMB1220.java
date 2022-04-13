package org.firstinspires.ftc.teamcode.Subsystems.Sensors;

import android.util.Log;

import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.io.PrintWriter;
import java.io.StringWriter;

public class CorrectedMB1220 extends MovingUltrasonicSensor {

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

    public CorrectedMB1220(HardwareMap h, String name, int smoothing, Facing f, Localizer l) {

        super(smoothing, f, l);
        mb1220 = h.analogInput.get(name);
        portNum = Integer.parseInt(mb1220.getConnectionInfo().split("; analog port ")[1]);
        this.v = VOLTAGE.THREE;

    }

    public CorrectedMB1220(HardwareMap h, String name, BulkRead bRead, int smoothing, Facing f, Localizer l) {

        super(smoothing, f, l);
        mb1220 = h.analogInput.get(name);
        this.bRead = bRead;
        portNum = Integer.parseInt(mb1220.getConnectionInfo().split("; analog port ")[1]);
        this.v = VOLTAGE.THREE;

    }

    public CorrectedMB1220(HardwareMap h, String name, VOLTAGE v, int smoothing, Facing f, Localizer l) {

        super(smoothing, f, l);
        mb1220 = h.analogInput.get(name);
        portNum = Integer.parseInt(mb1220.getConnectionInfo().split("; analog port ")[1]);
        this.v = v;

    }

    public CorrectedMB1220(HardwareMap h, String name, BulkRead bRead, VOLTAGE v, int smoothing, Facing f, Localizer l) {

        super(smoothing, f, l);
        mb1220 = h.analogInput.get(name);
        this.bRead = bRead;
        portNum = Integer.parseInt(mb1220.getConnectionInfo().split("; analog port ")[1]);
        this.v = v;

    }

    @Override
    public void updateInput() {

        try {

            super.add(2 * (bRead != null ? bRead.getAnalogValue(portNum) : mb1220.getVoltage())
                    / (v == VOLTAGE.THREE ? THREEV_POWER : FIVEV_POWER) * 0.393701d);
            super.updateInput();
            distance = super.getValue();

        }
        catch(Exception e) {

            StringWriter sw = new StringWriter();
            e.printStackTrace(new PrintWriter(sw));
            Log.e("Error: ", sw.toString());
            distance = ERROR_VALUE;

        }

    }

    public VOLTAGE getVoltage() {return v;}
    public void setVoltage(VOLTAGE v) {this.v = v;}

    public double getDistance(DistanceUnit u) {

        switch(u) {

            case INCH:
                return distance;
            case CM:
            default:
                return distance / 0.393701d;

        }

    }

}
