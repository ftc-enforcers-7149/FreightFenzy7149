package org.firstinspires.ftc.teamcode.Subsystems.Sensors;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Input;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.ValueTimer;

public class DistanceCorrection implements Input {

    private DistanceSensor sensorL, sensorR, sensorF;
    private ValueTimer<Double> lDist, rDist, fDist;
    private static double FIELD_X = 144, FIELD_Y = 144, F_OFFSET = 6.75 - 2.65, L_R_OFFSET = 7.5 - 0.3;

    private Alliance alliance;

    private boolean running;

    public DistanceCorrection(HardwareMap hardwareMap, String distLName, String distRName, String distFName,
                              Alliance alliance) {

        if (alliance == Alliance.BLUE) {
            sensorL = hardwareMap.get(DistanceSensor.class, distLName);
            lDist = new ValueTimer<Double>(0.0, 0) {
                @Override
                public Double readValue() {
                    return sensorL.getDistance(DistanceUnit.INCH) + L_R_OFFSET;
                }
            };
        }
        else {
            sensorR = hardwareMap.get(DistanceSensor.class, distRName);
            rDist = new ValueTimer<Double>(0.0, 0) {
                @Override
                public Double readValue() {
                    return sensorR.getDistance(DistanceUnit.INCH) + L_R_OFFSET;
                }
            };
        }

        sensorF = hardwareMap.get(DistanceSensor.class, distFName);
        fDist = new ValueTimer<Double>(0.0, 0) {
            @Override
            public Double readValue() {
                return sensorF.getDistance(DistanceUnit.INCH) + F_OFFSET;
            }
        };

        this.alliance = alliance;

        running = false;
    }

    public Vector2d correctPoseWithDist() {
        double distX = alliance.equals(Alliance.BLUE) ? lDist.getValue() : rDist.getValue();
        double distY = (FIELD_Y - fDist.getValue()) * (alliance.equals(Alliance.BLUE) ? 1 : -1);

        return new Vector2d(distX, distY);
    }

    public double getSideWall() {
        return alliance.equals(Alliance.BLUE) ? lDist.getValue() : rDist.getValue();
    }

    public double getFrontDistance() {
        return fDist.getValue();
    }

    public void startRunning() {
        if (alliance == Alliance.BLUE)
            lDist.startInput();
        else
            rDist.startInput();
        fDist.startInput();

        running = true;
    }

    public void stopRunning() {
        if (alliance == Alliance.BLUE)
            lDist.stopInput();
        else
            rDist.stopInput();
        fDist.stopInput();

        running = false;
    }

    @Override
    public void updateInput() {
        if (running) {
            if (alliance == Alliance.BLUE)
                lDist.updateInput();
            else
                rDist.updateInput();
            fDist.updateInput();
        }
    }

    @Override
    public void stopInput() {
        stopRunning();
    }
}
