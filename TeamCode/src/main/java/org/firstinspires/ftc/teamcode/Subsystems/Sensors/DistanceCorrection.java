package org.firstinspires.ftc.teamcode.Subsystems.Sensors;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Input;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.ValueTimer;

public class DistanceCorrection implements Input {

    public CorrectedMB1220 sensor;

    private static final double FIELD_X = 144, FIELD_Y = 144, F_OFFSET = 6, L_R_OFFSET = 0;

    private final Alliance alliance;

    private boolean running;

    public DistanceCorrection(HardwareMap hardwareMap, String distFLName, String distFRName, BulkRead bRead, Localizer l,
                              Alliance alliance) {

        if (alliance == Alliance.BLUE)
            sensor = new CorrectedMB1220(hardwareMap, distFRName, bRead, 9, MovingUltrasonicSensor.Facing.FRONT, l);
        else
            sensor = new CorrectedMB1220(hardwareMap, distFLName, bRead, 9, MovingUltrasonicSensor.Facing.FRONT, l);

        sensor.setQuartileSmoothing(true);

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
            /*double leftDist = lDist.getValue();
            double frontDist = fDist.getValue();*/

            double leftDist = sensor.getDistance(DistanceUnit.INCH); //DOESNT WORK
            double frontDist = sensor.getDistance(DistanceUnit.INCH);

            double robotX = leftDist * sensorMultiplier(angle);
            double robotY = 144 - frontDist * sensorMultiplier(angle);

            return new Vector2d(robotX, robotY);
        }
        else {
            double rightDist = sensor.getDistance(DistanceUnit.INCH);
            double frontDist = sensor.getDistance(DistanceUnit.INCH);

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

    //DOESNT WORK
    public double getSideWall() {
        return alliance.equals(Alliance.BLUE) ? sensor.getDistance(DistanceUnit.INCH) : sensor.getDistance(DistanceUnit.INCH);
    }

    public double getFrontDistance() {
        return sensor.getDistance(DistanceUnit.INCH) + 10;
    }

    public void startRunning() {
        sensor.startInput();

        running = true;
    }

    public void stopRunning() {
        sensor.stopInput();

        running = false;
    }

    @Override
    public void updateInput() {
        if (running) {
            sensor.updateInput();
        }
    }

    @Override
    public void stopInput() {
        stopRunning();
    }

    public void setQuartileSmoothing(boolean q) {
        sensor.setQuartileSmoothing(q);
    }
}
