package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
    private double offset;

    /**
     *Gyro constructor. initializes imu
     * @param hardwareMap hardwareMap
     */
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

        offset = 0;
    }

    //x acceleration of imu
    public double getXAccel(){
        return imu.getLinearAcceleration().xAccel;
    }

    //y acceleration of imu
    public double getYAccel(){
        return imu.getLinearAcceleration().yAccel;
    }

    /**
     * Gets the shortest distance between two angles.
     * @param destAngle Destination angle
     * @param heading   Current angle
     * @return
     */
    public double getDelta(double destAngle, double heading) {
        //ensures heading is under 360
        if (heading >= 360) {
            heading -= 360;
        }
        double delta;
        //finds delta when current heading is greater the the destination heading
        if (heading > destAngle) {
            delta = heading - destAngle;
            if (delta > 180) {
                return 360 - delta;
            }

            return -delta;
        }
        //finds delta when current heading is less than the the destination heading
        else {
            delta = destAngle - heading;
            if (delta > 180) {
                return -(360 - delta);
            }

            return delta;
        }
    }

    public void setOffset(double offset){
        this.offset = offset;
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
    public static double cvtTrigAng(double heading) {
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
     * adds a constant offset
     * to the current angle
     * @param yaw current angle
     * @param offset angle offset
     * @return
     */
    public double cvtOffset(double yaw, double offset){
        double retVal = yaw;

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
    public double getYaw() {return cvtOffset(cvtDegrees(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle),offset);}

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