package org.firstinspires.ftc.teamcode.Subsystems.Sensors;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Subsytem;
import org.firstinspires.ftc.teamcode.Subsystems.ValueTimer;

public class Positioning implements Subsytem {

    private ValueTimer<Double> distanceLeft, distanceRight;
    public Rev2mDistanceSensor distL, distR;

    private ValueTimer<Double> lightDetected;
    public RevColorSensorV3 lineDetector;

    public Positioning(HardwareMap hardwareMap, String distLName, String distRName, String lineName) {
        distL = hardwareMap.get(Rev2mDistanceSensor.class, distLName);
        distR = hardwareMap.get(Rev2mDistanceSensor.class, distRName);
        lineDetector = hardwareMap.get(RevColorSensorV3.class, lineName);

        distanceLeft = new ValueTimer<Double>() {
            @Override
            public Double readValue() {
                return distL.getDistance(DistanceUnit.INCH);
            }
        };

        distanceRight = new ValueTimer<Double>() {
            @Override
            public Double readValue() {
                return distR.getDistance(DistanceUnit.INCH);
            }
        };

        lightDetected = new ValueTimer<Double>() {
            @Override
            public Double readValue() {
                return lineDetector.getLightDetected();
            }
        };
    }

    public void start() {
        distanceLeft.start();
        distanceRight.start();
        lightDetected.start();
    }

    @Override
    public void update() {
        distanceLeft.update();
        distanceRight.update();
        lightDetected.update();
    }

    public double getLeftDistance() {
        return distanceLeft.getValue();
    }

    public double getRightDistance() {
        return distanceRight.getValue();
    }

    public double getLightDetected() {
        return lightDetected.getValue();
    }

    @Override
    public void stop() {
        distanceLeft.stop();
        distanceRight.stop();
        lightDetected.stop();
    }
}