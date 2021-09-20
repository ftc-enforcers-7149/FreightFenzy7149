package org.firstinspires.ftc.teamcode.Mattu.imuclass;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@TeleOp(name = "IMU Basics")
@Disabled
public class IMUBasics extends OpMode {

    private BNO055IMU imu;

    public void init() {
        //Set up imu parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        //Desired output units
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        //Calibration data. Leave this as is unless you recalibrate the IMU manually
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        //Logging data
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        //An acceleration integrator takes the acceleration data and integrates it into
        //velocity and position. However, the JustLoggingAccelerationIntegrator class only
        //logs the acceleration output, and does not attempt to calculate anything from it.
        //If you want to try to get velocity and position output, you need to either use
        //NaiveAccelerationIntegrator or create your own.
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        //Get the imu specified in the configuration of the robot
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //This starts the acceleration integrator if one is set
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    public void loop() {
        //Gets the angles of the IMU in degrees
        double angleZ = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double angleY = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle;
        double angleX = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle;

        telemetry.addData("Angle Z: ", angleZ);
        telemetry.addData("Angle Y: ", angleY);
        telemetry.addData("Angle X: ", angleX);
        telemetry.addLine();

        //Get the accelerations of the IMU in m/s/s
        double accelX = imu.getAcceleration().xAccel;
        double accelY = imu.getAcceleration().yAccel;
        double accelZ = imu.getAcceleration().zAccel;

        telemetry.addData("Accel X: ", accelX);
        telemetry.addData("Accel Y: ", accelY);
        telemetry.addData("Accel Z: ", accelZ);
    }
}