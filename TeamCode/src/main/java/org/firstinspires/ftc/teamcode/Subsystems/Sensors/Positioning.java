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

    private static final double L_OFFSET = 1.26;
    private static final double R_OFFSET = 0;

    private ValueTimer<Double> lightDetected;
    public RevColorSensorV3 lineDetector;

    private static final double LIGHT_THRESH = 0.09;

    public Positioning(HardwareMap hardwareMap, String distLName, String distRName, String lineName) {
        distL = hardwareMap.get(Rev2mDistanceSensor.class, distLName);
        distR = hardwareMap.get(Rev2mDistanceSensor.class, distRName);
        lineDetector = hardwareMap.get(RevColorSensorV3.class, lineName);

        distanceLeft = new ValueTimer<Double>(500) {
            @Override
            public Double readValue() {
                return distL.getDistance(DistanceUnit.INCH);
            }
        };

        distanceRight = new ValueTimer<Double>(500) {
            @Override
            public Double readValue() {
                return distR.getDistance(DistanceUnit.INCH);
            }
        };

        lightDetected = new ValueTimer<Double>(0) {
            @Override
            public Double readValue() {
                return lineDetector.getLightDetected();
            }
        };
    }

    @Override
    public void updateInput() {
        distanceLeft.updateInput();
        distanceRight.updateInput();
        lightDetected.updateInput();
    }

    public void startPositioning() {
        distanceLeft.startInput();
        distanceRight.startInput();
        lightDetected.startInput();
    }
    public void stopPositioning() {
        stopInput();
    }

    public double getLeftDistance() {
        return distanceLeft.getValue() - L_OFFSET;
    }
    public double getRightDistance() {
        return distanceRight.getValue() - R_OFFSET;
    }
    public boolean getLineDetected() {
        return lightDetected.getValue() >= LIGHT_THRESH;
    }

    @Override
    public void stopInput() {
        distanceLeft.stopInput();
        distanceRight.stopInput();
        lightDetected.stopInput();
    }
}
