package org.firstinspires.ftc.teamcode.Mattu.imuclass;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous(name = "Drive Straight Auto")
@Disabled
public class DriveStraightAuto extends LinearOpMode {

    private BNO055IMU imu;
    private DcMotor fLeft, fRight, bRight, bLeft;

    @Override
    public void runOpMode() throws InterruptedException {
        //Set up imu parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        //Initialize the motors, directions, and turn on braking at zero power
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");

        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);

        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        if (isStopRequested()) return;

        //Perform two simple drives
        //The robot should end facing the same direction it started facing (aka 0 degrees)
        driveStraight(1, 1500);
        strafeStraight(-1, 1500);
    }

    /**
     * Drives the robot straight forward or backward (depending on the power)
     * Uses the IMU to keep straight from wherever the robot started facing
     * @param speed The power to set to the motors when driving (-1, 1)
     * @param millis The amount of time to drive for, in milliseconds
     */
    private void driveStraight(double speed, double millis) {
        //The starting angle of the robot will be the target heading for the entire path
        double targetAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        //This will continue driving until the time specified has passed, or the program is stopped
        double startingTime = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - startingTime) < millis) {
            //Get (and add to telemetry) the current robot's angle
            double angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            telemetry.addData("Angle: ", angle);

            //Get the difference between the robot's angle and the target angle
            double difference = angle - targetAngle;
            if (difference < -180) difference += 360;
            else if (difference > 180) difference = 360 - difference;

            //This logic handles driving forward vs backward differently
            if (speed > 0) {
                //If the robot is driving forward and needs to turn to the right
                //Then reduce the speed of the right motors
                if (difference > 1) {
                    fLeft.setPower(speed);
                    fRight.setPower(speed - 0.2);
                    bLeft.setPower(speed);
                    bRight.setPower(speed - 0.2);
                }
                //If the robot is driving forward and needs to turn to the left
                //Then reduce the speed of the left motors
                else if (difference < -1) {
                    fLeft.setPower(speed - 0.2);
                    fRight.setPower(speed);
                    bLeft.setPower(speed - 0.2);
                    bRight.setPower(speed);
                }
                //If the robot is straight, drive normally
                else {
                    fLeft.setPower(speed);
                    fRight.setPower(speed);
                    bLeft.setPower(speed);
                    bRight.setPower(speed);
                }
            }
            else {
                //If the robot is driving backward and needs to turn to the right
                //Then reduce the (absolute) speed of the left motors
                if (difference > 1) {
                    fLeft.setPower(speed + 0.2);
                    fRight.setPower(speed);
                    bLeft.setPower(speed + 0.2);
                    bRight.setPower(speed);
                }
                //If the robot is driving backward and needs to turn to the left
                //Then reduce the (absolute) speed of the right motors
                else if (difference < -1) {
                    fLeft.setPower(speed);
                    fRight.setPower(speed + 0.2);
                    bLeft.setPower(speed);
                    bRight.setPower(speed + 0.2);
                }
                //If the robot is straight, drive normally
                else {
                    fLeft.setPower(speed);
                    fRight.setPower(speed);
                    bLeft.setPower(speed);
                    bRight.setPower(speed);
                }
            }
        }

        //Stop the robot after the time has passed
        fLeft.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);
    }

    /**
     * Drives the robot straight left or right (depending on the power)
     * Uses the IMU to keep straight from wherever the robot started facing
     * @param speed The power to set to the motors when driving (-1, 1)
     * @param millis The amount of time to strafe for, in milliseconds
     */
    private void strafeStraight(double speed, double millis) {
        //The starting angle of the robot will be the target heading for the entire path
        double targetAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        //This will continue driving until the time specified has passed, or the program is stopped
        double startingTime = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - startingTime) < millis) {
            //Get (and add to telemetry) the current robot's angle
            double angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            telemetry.addData("Angle: ", angle);

            //Get the difference between the robot's angle and the target angle
            double difference = angle - targetAngle;
            if (difference < -180) difference += 360;
            else if (difference > 180) difference = 360 - difference;

            //This logic handles strafing right vs left differently
            if (speed > 0) {
                //If the robot is strafing right and needs to turn to the right
                //Then reduce the (absolute) speed of the back motors
                if (difference > 1) {
                    fLeft.setPower(speed);
                    fRight.setPower(-speed);
                    bLeft.setPower(-speed + 0.2);
                    bRight.setPower(speed - 0.2);
                }
                //If the robot is strafing right and needs to turn to the left
                //Then reduce the (absolute) speed of the front motors
                else if (difference < -1) {
                    fLeft.setPower(speed - 0.2);
                    fRight.setPower(-speed + 0.2);
                    bLeft.setPower(-speed);
                    bRight.setPower(speed);
                }
                //If the robot is straight, strafe normally
                else {
                    fLeft.setPower(speed);
                    fRight.setPower(-speed);
                    bLeft.setPower(-speed);
                    bRight.setPower(speed);
                }
            }
            else {
                //If the robot is strafing left and needs to turn to the right
                //Then reduce the (absolute) speed of the front motors
                if (difference > 1) {
                    fLeft.setPower(speed + 0.2);
                    fRight.setPower(-speed - 0.2);
                    bLeft.setPower(-speed);
                    bRight.setPower(speed);
                }
                //If the robot is strafing left and needs to turn to the left
                //Then reduce the (absolute) speed of the back motors
                else if (difference < -1) {
                    fLeft.setPower(speed);
                    fRight.setPower(-speed);
                    bLeft.setPower(-speed - 0.2);
                    bRight.setPower(speed + 0.2);
                }
                //If the robot is straight, strafe normally
                else {
                    fLeft.setPower(speed);
                    fRight.setPower(-speed);
                    bLeft.setPower(-speed);
                    bRight.setPower(speed);
                }
            }
        }

        //Stop the robot after the time has passed
        fLeft.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);
    }
}
