package org.firstinspires.ftc.teamcode.Subsystems.Sensors;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Input;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.ValueTimer;

public class DistanceCorrection implements Input {

    public Rev2mDistanceSensor sensorL, sensorR, sensorF;
    private ValueTimer<Double> lDist, rDist, fDist;
    private static final double FIELD_X = 144, FIELD_Y = 144, F_OFFSET = 6.75 - 2.65, L_R_OFFSET = 7.5 - 0.3;

    private final Alliance alliance;

    private boolean running;

    public DistanceCorrection(HardwareMap hardwareMap, String distLName, String distRName, String distFName,
                              Alliance alliance) {

        if (alliance == Alliance.BLUE) {
            sensorL = hardwareMap.get(Rev2mDistanceSensor.class, distLName);
            lDist = new ValueTimer<Double>(0.0, 0) {
                @Override
                public Double readValue() {
                    return sensorL.getDistance(DistanceUnit.INCH) + L_R_OFFSET;
                }
            };
        }
        else {
            sensorR = hardwareMap.get(Rev2mDistanceSensor.class, distRName);
            rDist = new ValueTimer<Double>(0.0, 0) {
                @Override
                public Double readValue() {
                    return sensorR.getDistance(DistanceUnit.INCH) + L_R_OFFSET;
                }
            };
        }

        sensorF = hardwareMap.get(Rev2mDistanceSensor.class, distFName);
        fDist = new ValueTimer<Double>(0.0, 0) {
            @Override
            public Double readValue() {
                return sensorF.getDistance(DistanceUnit.INCH) + F_OFFSET;
            }
        };

        this.alliance = alliance;

        running = false;
    }

    /**
     * Returns a robot position (x, y) based on its heading and distance sensor readings
     * Only works when in the appropriate warehouse
     * @param angle The robot's heading, in radians
     * @return A new robot position
     */
    public Vector2d correctPoseWithDist(double angle) {
        if (alliance == Alliance.BLUE) {
            double leftDist = lDist.getValue();
            double frontDist = fDist.getValue();

            double robotX = leftDist * sensorMultiplier(angle);
            double robotY = 144 - frontDist * sensorMultiplier(angle);

            return new Vector2d(robotX, robotY);
        }
        else {
            double rightDist = rDist.getValue();
            double frontDist = fDist.getValue();

            double robotX = rightDist * sensorMultiplier(angle);
            double robotY = frontDist * sensorMultiplier(angle) - 144;

            return new Vector2d(robotX, robotY);
        }
    }

    private double sensorMultiplier(double angle) {
        if (alliance == Alliance.BLUE)
            return Math.cos(Math.abs(Math.toRadians(90) - angle));
        else
            return Math.cos(Math.abs(Math.toRadians(270) - angle));
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

    public Rev2mDistanceSensor getSensorL() {
        return sensorL;
    }

    public Rev2mDistanceSensor getSensorR() {
        return sensorR;
    }

    public Rev2mDistanceSensor getSensorF() {
        return sensorF;
    }
}
