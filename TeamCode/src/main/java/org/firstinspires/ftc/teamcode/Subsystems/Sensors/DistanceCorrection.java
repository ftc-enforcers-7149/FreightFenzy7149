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

    // public Rev2mDistanceSensor sensorL, sensorR, sensorF;
    public CorrectedMB1220 sensorL, sensorR, sensorF;
    // private ValueTimer<Double> lDist, rDist, fDist;

    private static final double FIELD_X = 144, FIELD_Y = 144, F_OFFSET = 6, L_R_OFFSET = 0;

    private final Alliance alliance;

    private boolean running;

    public DistanceCorrection(HardwareMap hardwareMap, String distLName, String distRName, String distFName, BulkRead bRead, Localizer l,
                              Alliance alliance) {

        if (alliance == Alliance.BLUE) {
            // sensorL = hardwareMap.get(Rev2mDistanceSensor.class, distLName);
            //sensorL = new MaxbotixMB1220(hardwareMap, distLName, bRead, MaxbotixMB1220.VOLTAGE.THREE, 9);
            sensorL = new CorrectedMB1220(hardwareMap, distLName, bRead, 9, MovingUltrasonicSensor.Facing.LEFT, l);
            //sensorL.mb1220.resetDeviceConfigurationForOpMode();
            /*lDist = new ValueTimer<Double>(350.0, 200) {
                @Override
                public Double readValue() {
                    //return sensorL.getDistance(DistanceUnit.INCH) + L_R_OFFSET;
                    return sensorL.getDistance() * 0.393701d *//*cvt to cm*//* + L_R_OFFSET;
                }
            };*/
        }
        else {
            //sensorR = hardwareMap.get(Rev2mDistanceSensor.class, distRName);
            sensorR = new CorrectedMB1220(hardwareMap, distRName, bRead, 9, MovingUltrasonicSensor.Facing.RIGHT, l);
            //sensorR.mb1220.resetDeviceConfigurationForOpMode();
            /*rDist = new ValueTimer<Double>(350.0, 200) {
                @Override
                public Double readValue() {
                    //return sensorR.getDistance(DistanceUnit.INCH) + L_R_OFFSET;
                    return sensorR.getDistance() * 0.393701d *//*cvt to cm*//* + L_R_OFFSET;
                }
            };*/
        }

        // sensorF = hardwareMap.get(Rev2mDistanceSensor.class, distFName);
        sensorF = new CorrectedMB1220(hardwareMap, distLName, bRead, 9, MovingUltrasonicSensor.Facing.FRONT, l);
        //sensorF.mb1220.resetDeviceConfigurationForOpMode();
        /*fDist = new ValueTimer<Double>(350.0, 200) {
            @Override
            public Double readValue() {
                //return sensorF.getDistance(DistanceUnit.INCH) + F_OFFSET;
                return sensorF.getDistance() * 0.393701d *//*cvt to cm*//* + F_OFFSET;
            }
        };*/

        this.alliance = alliance;

        /*I2cDeviceSynch.ReadWindow oldW = sensorL.getDeviceClient().getReadWindow();
        I2cDeviceSynch.ReadWindow w = new I2cDeviceSynch.ReadWindow(oldW.getRegisterFirst(), oldW.getRegisterCount(), I2cDeviceSynch.ReadMode.ONLY_ONCE);
        sensorL.getDeviceClient().setReadWindow(w);*/

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

            double leftDist = sensorL.getDistance(DistanceUnit.INCH);
            double frontDist = sensorF.getDistance(DistanceUnit.INCH);

            double robotX = leftDist * sensorMultiplier(angle);
            double robotY = 144 - frontDist * sensorMultiplier(angle);

            return new Vector2d(robotX, robotY);
        }
        else {
            double rightDist = sensorR.getDistance(DistanceUnit.INCH);
            double frontDist = sensorF.getDistance(DistanceUnit.INCH);

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
        return alliance.equals(Alliance.BLUE) ? sensorL.getDistance(DistanceUnit.INCH) : sensorR.getDistance(DistanceUnit.INCH);
    }

    public double getFrontDistance() {
        return sensorF.getDistance(DistanceUnit.INCH);
    }

    public void startRunning() {
        /*if (alliance == Alliance.BLUE)
            lDist.startInput();
        else
            rDist.startInput();
        fDist.startInput();*/

        running = true;
    }

    public void startSideSensor() {
        /*if (alliance == Alliance.BLUE)
            lDist.startInput();
        else
            rDist.startInput();*/

        running = true;
    }

    public void startFrontSensor() {
        /*fDist.startInput();*/

        running = true;
    }

    public void stopRunning() {
        /*if (alliance == Alliance.BLUE)
            lDist.stopInput();
        else
            rDist.stopInput();
        fDist.stopInput();*/

        running = false;
    }

    @Override
    public void updateInput() {
        if (running) {
            if (alliance == Alliance.BLUE)
                sensorL.updateInput();
            else
                sensorR.updateInput();
            sensorF.updateInput();
        }
    }

    @Override
    public void stopInput() {
        stopRunning();
    }

    public void setSensorL(HardwareMap hardwareMap, String distLName) {
        sensorL.mb1220.resetDeviceConfigurationForOpMode();
        //sensorL.close();
        //sensorL = hardwareMap.get(Rev2mDistanceSensor.class, distLName);
    }
    public void setSensorR(HardwareMap hardwareMap, String distRName) {
        sensorR.mb1220.resetDeviceConfigurationForOpMode();
        //sensorR.close();
        //sensorR = hardwareMap.get(Rev2mDistanceSensor.class, distRName);
    }
    public void setSensorF(HardwareMap hardwareMap, String distFName) {
        sensorF.mb1220.resetDeviceConfigurationForOpMode();
        //sensorF.close();
        //sensorF = hardwareMap.get(Rev2mDistanceSensor.class, distFName);
    }

    public void setQuartileSmoothing(boolean q) {
        sensorF.setQuartileSmoothing(q);

        if (alliance == Alliance.BLUE)
            sensorL.setQuartileSmoothing(q);
        else
            sensorR.setQuartileSmoothing(q);
    }

    /*public Rev2mDistanceSensor getSensorL() {
        return sensorL;
    }

    public Rev2mDistanceSensor getSensorR() {
        return sensorR;
    }

    public Rev2mDistanceSensor getSensorF() {
        return sensorF;
    }*/

    public CorrectedMB1220 getSensorL() {
        return sensorL;
    }

    public CorrectedMB1220 getSensorR() {
        return sensorR;
    }

    public CorrectedMB1220 getSensorF() {
        return sensorF;
    }

}
