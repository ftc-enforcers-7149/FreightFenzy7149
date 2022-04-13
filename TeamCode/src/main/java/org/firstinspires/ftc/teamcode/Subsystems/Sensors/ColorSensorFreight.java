package org.firstinspires.ftc.teamcode.Subsystems.Sensors;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Input;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.ValueTimer;

import java.util.ArrayList;

public class ColorSensorFreight implements Input {

    private RevColorSensorV3 sensor;
    private ArrayList<Sensor> vals = new ArrayList<>();
    private Freight currentType;

    public ValueTimer<Double> distance/*, light*/;
    //public ValueTimer<Integer> alpha;
    public ValueTimer<Integer> red, green, blue;

    private ArrayList<ValueTimer> valueTimers;

    private static final double minDistance = 2;

    public enum Freight {

        NONE(94.5),
        BLOCK(50),
        DUCK(76),
        BALL(81),
        UNKNOWN(125);

        public double minHue;

        Freight(double hue) {this.minHue = hue;}

        public static Freight getType(double hue) {
            if (hue <= DUCK.minHue) return BLOCK;
            else if (hue <= BALL.minHue) return DUCK;
            else if (hue <= UNKNOWN.minHue) return BALL;
            else return UNKNOWN;
        }
    }

    public ColorSensorFreight(HardwareMap h, String name, int smoothing) {

        sensor = h.get(RevColorSensorV3.class, name);

        distance = new ValueTimer<Double>(0.0, 100) {
            @Override
            public Double readValue() {
                return sensor.getDistance(DistanceUnit.INCH);
            }
        };
//
//        light = new ValueTimer<Double>(0.0, 200) {
//            @Override
//            public Double readValue() {
//                return sensor.getLightDetected();
//            }
//        };
//
//        alpha = new ValueTimer<Integer>(0, 200) {
//            @Override
//            public Integer readValue() {
//                return sensor.alpha();
//            }
//        };

        red = new ValueTimer<Integer>(0, 100) {
            @Override
            public Integer readValue() {
                return sensor.red();
            }
        };

        green = new ValueTimer<Integer>(0, 100) {
            @Override
            public Integer readValue() {
                return sensor.green();
            }
        };

        blue = new ValueTimer<Integer>(0, 100) {
            @Override
            public Integer readValue() {
                return sensor.blue();
            }
        };

        valueTimers = new ArrayList<ValueTimer>();
        valueTimers.add(distance);
//        valueTimers.add(light);
//        valueTimers.add(alpha);
        valueTimers.add(red);
        valueTimers.add(green);
        valueTimers.add(blue);

        // adds in order: light, a, h, s, v, distance
        for(int i = 0; i < 6; i++) vals.add(new Sensor(smoothing));
    }

    @Override
    public void updateInput() {
        for (ValueTimer valueTimer : valueTimers)
            valueTimer.updateInput();

        double[] curHSV = rgbToHSV(red.getValue(), green.getValue(), blue.getValue());

        vals.get(0).add(curHSV[0]);                             // hue
        vals.get(1).add(curHSV[1]);                             // saturation
        vals.get(2).add(curHSV[2]);                             // value
  //    vals.get(3).add(light.getValue());                      // light
  //    vals.get(4).add(alpha.getValue());                      // alpha
        vals.get(5).add(distance.getValue());                   // distance

        for(Sensor s : vals) s.updateInput();

        if(getSaturation() <= 0.2) {
            currentType = /*getHue() < Freight.NONE.minHue ? Freight.DUCK : */Freight.NONE; // sometimes saturation gets really low with
        }                                                                               // an intake duck, but hue can clue us in
        else currentType = Freight.getType(getHue());

    }

    public static double[] rgbToHSV(double r, double g, double b) {

        // values need to be in a range from 0 to 1 so we normalize em
        r /= 255;
        g /= 255;
        b /= 255;

        // V happens to be our max value our max value.
        double V = Math.max(r, Math.max(g, b));
        double min = Math.min(r, Math.min(g, b));


        // determining whether r, g, or b is our max (a surprise tool that will help us later)
        char max = (V == r) ? 'r' : (V == g) ? 'g' : 'b';

        // H is automatically zero because it's one less else statement
        double H = 0;

        // Checks if max - min isn't zero, bc if it is, H is zero
        if(V - min != 0) switch(max) {

            case 'r':
                H = (g - b) / (V - min);
                break;
            case 'g':
                H = 2 + (b - r) / (V - min);
                break;
            case 'b':
                H = 4 + (r - g) / (V - min);
                break;

        }

        // No matter what the max value is, it gets multiplied by 60 degrees :sunglasses:
        H *= 60;

        double S = (V == 0) ? 0 : (V - min);

        // returns an array of H, S, and V
        return new double[]{H, S, V};

    }

    public double getLight() {          return vals.get(3).getValue(); }
    public double getAlpha() {          return vals.get(4).getValue(); }
    public double getHue() {            return vals.get(0).getValue(); }
    public double getSaturation() {     return vals.get(1).getValue(); }
    public double getValue() {          return vals.get(2).getValue(); }
    public double getDistance() {       return vals.get(5).getValue(); }

    public Freight getCurrentType() { return currentType; }

    @Override
    public void startInput() {
        for (ValueTimer valueTimer : valueTimers)
            valueTimer.startInput();
    }

    @Override
    public void stopInput() {
        for (ValueTimer valueTimer : valueTimers)
            valueTimer.stopInput();
    }
}
