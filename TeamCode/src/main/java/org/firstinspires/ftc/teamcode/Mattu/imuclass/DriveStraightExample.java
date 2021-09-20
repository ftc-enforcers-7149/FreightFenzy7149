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

@TeleOp(name = "Drive Straight Example")
@Disabled
public class DriveStraightExample extends OpMode {

    private BNO055IMU imu;
    private DcMotor fLeft, fRight, bRight, bLeft;

    public void init() {
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
    }

    public void loop() {
        //Gets the heading of the robot in degrees
        double angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        //Get trigger inputs
        double rtrigger = gamepad1.right_trigger;
        double ltrigger = gamepad1.left_trigger;

        double setPower = 0;

        //If the right trigger is pressed, the robot will go forward
        //If the left trigger is pressed, the robot will go backward
        //If neither are pressed, the robot won't move
        if (rtrigger > 0.1) {
            setPower = rtrigger;
        } else if (ltrigger > 0.1) {
            setPower = -ltrigger;
        }

        //If the robot is going forward
        if (setPower > 0) {

            //If the robot is too far to the left (of 0 degrees), turn to the right
            //This can be achieved by either adding power to the left motors, or subtracting
            //from the right motors. We opt for subtracting in this case so we don't add 0.2
            //to the max input of 1. setPower(1.2) won't actually spin the motors faster.
            if (angle > 1) {
                fLeft.setPower(setPower);
                fRight.setPower(setPower - 0.2);
                bLeft.setPower(setPower);
                bRight.setPower(setPower - 0.2);

            //If the robot is too far to the right (of 0 degrees), turn to the left
            //So, subtract power from the left motors
            } else if (angle < -1) {
                fLeft.setPower(setPower - 0.2);
                fRight.setPower(setPower);
                bLeft.setPower(setPower - 0.2);
                bRight.setPower(setPower);
            }

            //If the robot is straight forward (towards 0 degrees), just drive normally
            else {
                fLeft.setPower(setPower);
                fRight.setPower(setPower);
                bLeft.setPower(setPower);
                bRight.setPower(setPower);
            }
        }

        //If the robot is going backward
        else if (setPower < 0) {

            //If the robot is too far to the left (of 0 degrees), turn to the right
            //This can be achieved by either adding power to the left motors, or subtracting
            //from the right motors. We opt for adding in this case so we don't subtract 0.2
            //from the max input of -1. setPower(-1.2) won't actually spin the motors faster.
            if (angle > 1) {
                fLeft.setPower(setPower + 0.2);
                fRight.setPower(setPower);
                bLeft.setPower(setPower + 0.2);
                bRight.setPower(setPower);

                //If the robot is too far to the right (of 0 degrees), turn to the left
                //So, add power from the right motors
            } else if (angle < -1) {
                fLeft.setPower(setPower);
                fRight.setPower(setPower + 0.2);
                bLeft.setPower(setPower);
                bRight.setPower(setPower + 0.2);
            }

            //If the robot is straight forward (towards 0 degrees), just drive normally
            else {
                fLeft.setPower(setPower);
                fRight.setPower(setPower);
                bLeft.setPower(setPower);
                bRight.setPower(setPower);
            }
        }

        //If neither trigger is pressed, don't move
        else {
            fLeft.setPower(0);
            fRight.setPower(0);
            bLeft.setPower(0);
            bRight.setPower(0);
        }
    }

    public void stop() {
        fLeft.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);
    }
}
