package org.firstinspires.ftc.teamcode.Subsystems.Sensors;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Input;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.ValueTimer;

public class Positioning implements Input {

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

    @Override
    public void start() {
        distanceLeft.start();
        distanceRight.start();
        lightDetected.start();
    }

    @Override
    public void updateInput() {
        distanceLeft.updateInput();
        distanceRight.updateInput();
        lightDetected.updateInput();
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
