package org.firstinspires.ftc.teamcode.Odometry.Util;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

public class Gyroscope {
    //IMU variables
    private BNO055IMU imu;
    private Orientation angles;
    private Telemetry telemetry;

    /**
     *Gyro constructor. initializes imu
     * @param telemetry telemetry
     * @param hardwareMap hardwareMap
     */
    public Gyroscope(Telemetry telemetry, HardwareMap hardwareMap){
        //Set up imu parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        this.telemetry = telemetry;

        /*this.telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
        });*/

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    public double getXAccel(){
        return imu.getLinearAcceleration().xAccel /* 39.3701*/;
    }

    public double getYAccel(){
        return imu.getLinearAcceleration().yAccel /* 39.3701*/;
    }

    /**
     * Gets the shortest distance between two angles.
     * @param destAngle Destination angle
     * @param heading   Current angle
     * @return
     */
    public double getDelta(double destAngle, double heading) {
        if (heading >= 360) {
            heading -= 360;
        }
        double delta;
        if (heading > destAngle) {
            delta = heading - destAngle;
            if (delta > 180) {
                return 360 - delta;
            }

            return -delta;
        }
        else {
            delta = destAngle - heading;
            if (delta > 180) {
                return -(360 - delta);
            }

            return delta;
        }
    }

    /**
     * Gets the shortest distance between two angles.
     * @param destAngle Destination angle
     * @param heading   Current angle
     * @return
     */
    public double getRelDelta(double destAngle, double heading) {
        return (heading-destAngle);
    }



    /**
     * converts gyro degrees from -180 to 180 to be 0 to 360
     * @param heading
     * @return
     */
    public double cvtDegrees(double heading) {
        if (heading < 0) {
            return 360 + heading;
        } else {
            return heading;
        }
    }

    /**
     * Converts degrees to work with sine and cosine
     * @param heading
     * @return
     */
    public double cvtTrigAng(double heading) {
        if (heading >= 0 && heading < 90) {
            return -heading + 90;
        }
        return -heading + 450;
    }

    public double cvtRelativeAng(double heading,double initAng){
        double retVal;
        if (heading < 0) {
            retVal =  360 + heading;
        } else {
            retVal =  heading;
        }

        return (retVal+180+initAng)%360;
    }

    /**
     * returns raw yaw value from gyro
     * @return
     */
    public double getRawYaw(){
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    /**
     * returns degrees in 0-360 degree format
     * @return
     */
    public double getYaw() {return cvtDegrees(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);}

    /**
     * returns degrees in 0-360 degree format, with straight being 90 degrees instead of 0
     * @return
     */
    public double getTrigYaw() {return cvtTrigAng(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);}

    /**
     *
     */
    public double getRelativeYaw(double initAng){return cvtRelativeAng(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle,initAng);}


    /**
     * method needed for gyro
     * @param angleUnit
     * @param angle
     * @return
     */
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    /**
     * method needed for gyro
     * @param degrees
     * @return
     */
    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}