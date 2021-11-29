package org.firstinspires.ftc.teamcode.Subsystems.Sensors;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Input;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.ValueTimer;

public class Gyroscope implements Input {

    //IMU sensor
    public BNO055IMU imu;
    private ValueTimer<Float> yawReading, angVelReading;

    private double rawYaw, yaw, angVel;
    private double offset;

    public Gyroscope(HardwareMap hardwareMap){
        //Set up imu parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        yawReading = new ValueTimer<Float>() {
            @Override
            public Float readValue() {
                return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            }
        };
        angVelReading = new ValueTimer<Float>() {
            @Override
            public Float readValue() {
                return imu.getAngularVelocity().zRotationRate;
            }
        };

        offset = 0;
    }

    @Override
    public void start() {
        yawReading.start();
        angVelReading.start();
    }

    @Override
    public void updateInput() {
        yawReading.updateInput();
        angVelReading.updateInput();

        rawYaw = yawReading.getValue();
        yaw = cvtOffset(denormalizeDegrees(rawYaw), offset);
        angVel = angVelReading.getValue();
    }

    @Override
    public void stop() {
        yawReading.stop();
        angVelReading.stop();
    }

    /**
     * Set constant offset for the imu
     * Only applies to getYaw() function
     * @param offset Angle offset, in degrees [-inf,inf]
     */
    public void setOffset(double offset){
        this.offset = offset;
    }

    /**
     * Converts gyro degrees from -180 to 180 to be 0 to 360
     * @param angle Angle to convert
     * @return
     */
    public double denormalizeDegrees(double angle) {
        if (angle < 0) {
            return 360 + angle;
        } else {
            return angle;
        }
    }

    /**
     * Converts degrees to unit circle
     * @param angle Angle to convert
     * @return
     */
    public static double cvtTrigAng(double angle) {
        if (angle >= 0 && angle < 90) {
            return -angle + 90;
        }
        return -angle + 450;
    }

    /**
     * Adds a constant offset to the current angle
     * @param angle Current angle
     * @param offset Angle offset
     * @return
     */
    public double cvtOffset(double angle, double offset){
        double retVal = angle;

        retVal += offset;
        //makes sure angle is under 360 degrees
        if (retVal >= 360) {
            retVal -= 360;
        }
        else if (retVal < 0) {
            retVal += 360;
        }

        return retVal;
    }

    /**
     * @return Raw yaw value from gyro, in degrees
     */
    public double getRawYaw() {
        return rawYaw;
    }

    /**
     * @return Degrees in 0-360 degree format
     */
    public double getYaw() {
        return yaw;
    }

    /**
     * @return Angular velocity, in degrees
     */
    public double getAngVel() {
        return angVel;
    }

    /**
     * Makes a new sensor read before returning
     * @return Raw yaw value from gyro, in degrees
     */
    public double getNewRawYaw() {
        updateInput();
        return rawYaw;
    }

    /**
     * Makes a new sensor read before returning
     * @return Degrees in 0-360 degree format
     */
    public double getNewYaw() {
        updateInput();
        return yaw;
    }

    /**
     * Makes a new sensor read before returning
     * @return Angular velocity, in degrees
     */
    public double getNewAngVel() {
        updateInput();
        return angVel;
    }
}