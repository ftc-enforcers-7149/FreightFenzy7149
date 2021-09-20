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

@TeleOp(name = "Field Oriented")
@Disabled
public class FieldOriented extends OpMode {

    /*
     * Motors and their respective "velocities" (in reality just their motor powers)
     */
    private DcMotor fLeft, fRight, bLeft, bRight;
    private double vFL, vFR, vBL, vBR;

    /*
     * The IMU object is used to get the robot's angle
     * Angle (duh) stores said angle
     * Offset is used to correct for if the imu gains error throughout a match
     */
    private BNO055IMU imu;
    private double angle, offset;

    /*
     * The limit is used to make sure the drive motors don't go over a set value
     * For instance, the motor power must be <= 1 or it will cause an error
     */
    private double lim;

    /*
     * These are the Gamepad input variables
     */
    private double leftX, leftY, rightX;
    private boolean resetAngle;

    @Override
    public void init() {

        /*
         * Initializing the motors
         */
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");

        /*
         * Depending on your setup, different motors might need to be reversed
         * It could be both the right motors, both the front motors, etc.
         */

        /*
         * You can test this by driving all motors forward with a positive power (0.5),
         * and seeing which ones go backwards.
         */

        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);

        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ////////////////////////////////////////////////////////////////////////////////////////////

        /*
         * Initializing the imu and variables, in their own respective functions
         */
        initializeIMU();
        initialzeVars();
    }

    @Override
    public void loop() {
        /*
         * In loop, we first need to get our inputs
         * The way this will control is that the left joystick moves the robot around,
         * and the right joystick (only the x axis) turns the robot
         */
        leftX = gamepad1.left_stick_x;
        leftY = gamepad1.left_stick_y;
        rightX = gamepad1.right_stick_x;

        ////////////////////////////////////////////////////////////////////////////////////////////

        /*
         * The resetAngle button is used to fix the robot's front, in case the imu
         * accumulated error, or the robot didn't start the match straight
         */
        resetAngle = gamepad1.y;

        /*
         * Getting the robot angle
         */
        angle = getRawYaw();

        /*
         * Set the offset to the current angle when needed
         * By doing this, it effectively makes whatever direction the robot
         * was facing the "front".
         */
        if (resetAngle) {
            offset = angle;
        }

        ////////////////////////////////////////////////////////////////////////////////////////////

        /*
         * r is the radius of the circle formed by the left x and y joystick values
         * This will be used to scale the speed of the robot because the
         * farther out the joystick is pushed, the higher this value will be.
         *
         * The reason we multiply by the sqrt(2) is because the output of sine and cosine when
         * going straight will be sqrt(2)/2, not 1, which will be mentioned again later. Multiplying
         * by sqrt(2) will make the straight forward power 1, and while diagonal will be >1, we
         * will correct for that later.
         */
        double r = Math.hypot(leftX, leftY) * Math.sqrt(2);

        ////////////////////////////////////////////////////////////////////////////////////////////

        /*
         * This robot angle is not the angle the robot is facing. It is the angle in which the robot
         * needs to drive relative to its front, in order to drive at the field-relative angle
         * specified by the driver.
         */

        double robotAngle = Math.atan2(leftY, leftX) - Math.toRadians(cvtDegrees(angle - offset)) - 3 * Math.PI / 4;

        /*
         * Taking this step-by-step, we'll start with the atan2() function. Math.atan2 finds the
         * "arc tangent" of the specified lengths (it should be y first, x second).
         * What this gets is the angle the joystick is pointed in, or the field-relative
         * angle we want the robot to drive in.
         */

        /*
         * Next, we use a cvtDegrees() function. This is because the sin(), cos(), and atan2() functions
         * we use depend on a slightly different angle format than a normal angle circle.
         * It's known as a unit circle, where 0 degrees is at the right, 90 is at the top, and so on.
         */

        /*
         * Finally, we offset by the term a multiple Math.PI / 4, which is 45 degrees in radians.
         * We need to shift by 45 degrees in order to drive the mecanum wheels properly
         * This angle makes it so that when going straight, we don't use sin(0)=0 and cos(0) = 1,
         * but rather sin(45) = 0.707... and cos(45) = 0.707...
         */

        /*
         * Let's imagine a test case, where the robot is turned 90 degrees to the right,
         * and the driver is pushing the left joystick straight forward
         */

        /* We'll discuss the angles as if they were in degrees
         * The first portion will have a value of 90, because the joystick is pushed straight forward
         * The next portion will have a value of 0, because it is 90 degrees, converted to the unit
         * circle (0 is the same as 360)
         * The last portion will always have a value of 135.
         * The result of all of this is -45 degrees. In terms of how this value will get used,
         * this is the exact value we want for this case.
         */

        ////////////////////////////////////////////////////////////////////////////////////////////

        /*
         * This part consists of two main parts. The first part is the sin and cosine functions.
         * These functions determine the power distribution based on the angle the robot must go in.
         * r is a weight to make these values smaller if the joystick is not pushed as far.
         */

        /*
         * Then, we add in turning weight. Imagine that the joystick is pushed to the right,
         * which means the robot should turn clockwise. When turning clockwise, the left wheels
         * go forward, and the right wheels go backward.
         * So to make the input relate to the output, we add to the left powers and subtract from
         * the right powers
         */

        vFL = r * Math.sin(robotAngle) + rightX;
        vFR = r * Math.cos(robotAngle) - rightX;
        vBL = r * Math.cos(robotAngle) + rightX;
        vBR = r * Math.sin(robotAngle) - rightX;

        ////////////////////////////////////////////////////////////////////////////////////////////

        /*
         * This portion of code does the following:
         * 1. It finds the highest (absolute) power
         * 2. It determines if this value is within the range of the limit, set by lim
         * 3. Proportionally adjusts all motor powers so that the max power does not exceed lim
         */

        double max = Math.max(
                Math.max(
                        Math.abs(vFL),
                        Math.abs(vFR)),
                Math.max(
                        Math.abs(vBL),
                        Math.abs(vBR))
        );

        if (max > lim) {
            vFL /= max * (1 / lim);
            vFR /= max * (1 / lim);
            vBL /= max * (1 / lim);
            vBR /= max * (1 / lim);
        }

        ////////////////////////////////////////////////////////////////////////////////////////////

        /*
         * Sets powers to each motor
         */
        fLeft.setPower(vFL);
        fRight.setPower(vFR);
        bLeft.setPower(vBL);
        bRight.setPower(vBR);

        ////////////////////////////////////////////////////////////////////////////////////////////

        /*
         * Telemtry output
         */
        telemetry.addData("Heading", angle - offset);
    }

    @Override
    public void stop() {
        fLeft.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);
    }

    /**
     * This converts degrees to work with sine and cosine.
     * The equations were made in Desmos by plotting certain points (input, output)
     * Equation 1: y = -x + 90
     * Equation 2: y = -x + 450
     * @param heading The robot heading in degrees
     * @return The fixed heading in degrees
     */
    private double cvtDegrees(double heading) {
        if (heading >= 0 && heading < 90) {
            return -heading + 90;
        }
        return -heading + 450;
    }

    /**
     * The Z axis is the correct axis for when the Control Hub or Expansion Hub
     * is flat and parallel to the ground. It should be centered in the robot
     * for best readings
     * @return Robot angle in degrees
     */
    private double getRawYaw() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    /**
     * Start up the imu so we can get the robot angle
     */
    private void initializeIMU() {
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
    }

    /**
     * Initialize all variables to their starting values
     * Mainly, we set the limit here
     */
    private void initialzeVars() {
        vFL = 0;
        vFR = 0;
        vBL = 0;
        vBR = 0;

        angle = 0;
        offset = 0;

        lim = 1;

        leftX = 0;
        leftY = 0;
        rightX = 0;

        resetAngle = false;
    }
}