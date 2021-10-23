package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Odometry.DriveWheels.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.BulkRead;
import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;

import static org.firstinspires.ftc.teamcode.Subsystems.FixedRoadrunner.createPose2d;

public abstract class TeleOp_Base extends OpMode {

    //Drive
    protected MecanumDrive drive;
    protected DcMotorEx fLeft, fRight, bLeft, bRight;
    protected Gyroscope gyro;
    private boolean initializedMotors = false, initializedDrive = false, initializedGyro = false;

    //Control objects
    protected BulkRead bReadCH, bReadEH;
    private boolean hasCH, hasEH;

    //State machine logic
    protected double leftX, leftY, rightX, rightY;
    protected double lastLeftX, lastLeftY, lastRightX, lastRightY;
    protected double time, lastTime, startTimeL, startTimeR, startTime;

    //Headless
    protected double offset, lim;

    //Initialization
    protected void initializeDrive() {
        if (!initializedDrive) {
            fLeft = hardwareMap.get(DcMotorEx.class, "fLeft");
            fRight = hardwareMap.get(DcMotorEx.class, "fRight");
            bLeft = hardwareMap.get(DcMotorEx.class, "bLeft");
            bRight = hardwareMap.get(DcMotorEx.class, "bRight");

            fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            fRight.setDirection(DcMotorSimple.Direction.FORWARD);
            bLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            bRight.setDirection(DcMotorSimple.Direction.FORWARD);

            fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            fRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            bLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            bRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        else {
            fLeft = drive.fLeft;
            fRight = drive.fRight;
            bLeft = drive.bLeft;
            bRight = drive.bRight;
        }

        initializedMotors = true;
    }
    protected void initializeBulkRead() {
        try {
            bReadCH = new BulkRead(hardwareMap, "Control Hub");
            hasCH = true;
        } catch (Exception e) {
            hasCH = false;
        }
        try {
            bReadEH = new BulkRead(hardwareMap, "Expansion Hub");
            hasEH = true;
        } catch (Exception e) {
            hasEH = false;
        }
    }
    protected void initializeGyro() {
        if (!initializedDrive)
            gyro = new Gyroscope(hardwareMap);
        else
            gyro = drive.gyro;

        initializedGyro = true;
    }
    protected void initializeOdometry() throws Exception {
        if (!hasCH) throw new Exception("Missing \"Control Hub\". Check configuration file naming");
        if (initializedMotors && initializedGyro)
            drive = new MecanumDrive(hardwareMap, bReadCH, fLeft, fRight, bLeft, bRight, gyro);
        else if (initializedMotors)
            drive = new MecanumDrive(hardwareMap, bReadCH, fLeft, fRight, bLeft, bRight);
        else if (initializedGyro)
            drive = new MecanumDrive(hardwareMap, bReadCH, gyro);
        else
            drive = new MecanumDrive(hardwareMap, bReadCH);
        drive.setPoseEstimate(createPose2d(0, 0, 0));

        initializedDrive = true;
    }
    protected void initializeVars() {
        lastLeftX = 0; lastLeftY = 0; lastRightX = 0; lastRightY = 0;
        offset = 0; lim = 1;
    }

    //Loop updates
    protected void updateBulkRead() {
        if (hasCH) bReadCH.update();
        if (hasEH) bReadEH.update();
    }

    //Driving
    protected void driveArcade() {
        //Simple arcade math
        if (leftY != lastLeftY && leftX != lastLeftX && rightX != lastRightX) {
            double vFL = -leftY + leftX - rightX;
            double vFR = -leftY - leftX + rightX;
            double vBL = -leftY - leftX - rightX;
            double vBR = -leftY + leftX + rightX;

            //Getting the max value can assure that no motor will be set to a value above a certain point.
            double max = Math.max(Math.max(Math.abs(vFL), Math.abs(vFR)), Math.max(Math.abs(vBL), Math.abs(vBR)));

            //In this case, no motor can go above lim power by scaling them all down if such a thing might occur.
            if (max > lim) {
                vFL /= max / lim;
                vFR /= max / lim;
                vBL /= max / lim;
                vBR /= max / lim;
            }

            setMotorPowers(vFL, vFR, vBL, vBR);
        }
    }
    protected void driveHeadless(double angle, boolean reset) {
        if (reset) {
            offset = angle;
        }

        if ((leftY == 0 && lastLeftY != 0) &&
                (leftX == 0 && lastLeftX != 0) &&
                (rightX == 0 && lastRightX != 0)) {
            setMotorPowers(0, 0, 0, 0);
            return;
        }

        double r = Math.hypot(leftX, leftY) * Math.sqrt(2);
        double robotAngle = Math.atan2(leftY, leftX) - Math.toRadians(Gyroscope.cvtTrigAng(angle - offset)) - 3 * Math.PI / 4;

        double vFL = r * Math.sin(robotAngle) + rightX;
        double vFR = r * Math.cos(robotAngle) - rightX;
        double vBL = r * Math.cos(robotAngle) + rightX;
        double vBR = r * Math.sin(robotAngle) - rightX;

        double max = Math.max(
                Math.max(
                        Math.abs(vFL),
                        Math.abs(vFR)),
                Math.max(
                        Math.abs(vBL),
                        Math.abs(vBR))
        );

        if (max > lim) {
            vFL /= max / lim;
            vFR /= max / lim;
            vBL /= max / lim;
            vBR /= max / lim;
        }

        setMotorPowers(vFL, vFR, vBL, vBR);
    }

    protected void driveAccelHeadless(double angle, boolean reset, double accelTime, double turnFactor, boolean disregard) {

        // Resets the robot's angle
        if (reset) {
            offset = angle;
        }

        // A little bit of fun math to determine whether we've made a significant change in joystick position.
        // Have fun messing around with this to your liking. This is a complete guesstimate.

        if(Math.abs(leftX - lastLeftX) >= .15 || Math.abs(leftY - lastLeftY) >= .15) startTime = System.currentTimeMillis();

        // If our controllers are zeroed, set the motor powers to zero

        if ((leftY == 0 && lastLeftY != 0) &&
                (leftX == 0 && lastLeftX != 0) &&
                (rightX == 0 && lastRightX != 0)) {
            setMotorPowers(0, 0, 0, 0);
            return;
        }

        // Ratio for the wheels

        double r = Math.hypot(leftX, leftY) * Math.sqrt(2);

        //Robot angle

        double robotAngle = Math.atan2(leftY, leftX) - Math.toRadians(Gyroscope.cvtTrigAng(angle - offset)) - 3 * Math.PI / 4;

        // Sine and cosine factors. These ramp based on a time-based acceleration curve.

        double sinFactor = (time - startTime <= accelTime) && !disregard ? ((time - startTime) / accelTime) * Math.sin(robotAngle) : Math.sin(robotAngle);
        double cosFactor = (time - startTime <= accelTime) && !disregard ? ((time - startTime) / accelTime) * Math.cos(robotAngle) : Math.cos(robotAngle);

        // Sets the motor powers

        double vFL = r * sinFactor + rightX * turnFactor;
        double vFR = r * cosFactor - rightX * turnFactor;
        double vBL = r * cosFactor + rightX * turnFactor;
        double vBR = r * sinFactor - rightX * turnFactor;

        // Calculates the maximum moter powers

        double max = Math.max(
                Math.max(
                        Math.abs(vFL),
                        Math.abs(vFR)),
                Math.max(
                        Math.abs(vBL),
                        Math.abs(vBR))
        );

        // Normalizes the wheel powers

        if (max > lim) {
            vFL /= max / lim;
            vFR /= max / lim;
            vBL /= max / lim;
            vBR /= max / lim;
        }

        // Sets the motor powers

        setMotorPowers(vFL, vFR, vBL, vBR);
    }

    protected void driveTank() {
        if (leftY != lastLeftY || rightY != lastRightY) {
            double vL = leftY;
            double vR = rightY;

            double max = Math.max(Math.abs(vL), Math.abs(vR));

            if (max > lim) {
                vL /= max / lim;
                vR /= max / lim;
            }

            setMotorPowers(vL, vR, vL, vR);
        }
    }

    protected void driveAccelTank(double accelTime, boolean disregard) {

        // Checks to see if we need to update anything.

        if (leftY != lastLeftY || rightY != lastRightY || time - startTimeL <= accelTime || time - startTimeR <= accelTime) {

            // Checks for significant change in left joystick position. This value is an estimate.

            if(Math.abs(leftY - lastLeftY) >= .15) {

                startTimeL = System.currentTimeMillis();

            }

            // Checks for significant change in right joystick position. This value is an estimate.

            if(Math.abs(rightY - lastRightY) >= .15) {

                startTimeR = System.currentTimeMillis();

            }

            // Figures out if we're turning or not, and creates a multiplier to slow us down if so.

            double turnMult = Math.abs(leftY - rightY) > .1 ? .6 : 1;

            // The fun bit. If we're not disregarding acceleration and we haven't gone over our ramp-up time, we set
            // power based on the time elapsed in the acceleration, standardized against the acceleration time.

            double vL = (time - startTimeL <= accelTime) && !disregard ? leftY * (time - startTimeL) / accelTime : leftY;
            double vR = (time - startTimeR <= accelTime) && !disregard ? rightY * (time - startTimeR) / accelTime : rightY;

            // Ensures we don't start at 0 power.
            //TODO: tune
            if(vL < .15) vL = .15;
            if(vR < .15) vR = .15;

            // If one wheel has already ramped up and we're not turning, we ramp the other motor up with it in proportion.

            if (Math.abs(vL) >= turnMult + .05) vR = rightY * (vL / leftY);
            else if (Math.abs(vR) >= turnMult + .05) vL = leftY * (vR / rightY);

            // Calculates the maximum arbitrary power value.

            double max = Math.max(Math.abs(vL), Math.abs(vR));

            // Standardizes the power on a scale of 1.

            if (max > lim) {
                vL /= max / lim;
                vR /= max / lim;
            }

            // Sets motor powers.

            setMotorPowers(vL * turnMult, vR * turnMult, vL * turnMult, vR * turnMult);
        }
    }

    //State machine logic
    protected abstract void getInput();
    protected abstract void updateStateMachine();

    //Useful functions

    /**
     * Curves an input so that 0->0, -1->-1, and 1->1,
     * but in between values curve slowly up by input^exp
     * @param input The function input
     * @param exp The exponent for the curve
     * @return The result of curving the input
     */
    protected double curveInput(double input, double exp) {
        return Math.signum(input) * Math.abs(Math.pow(input, exp));
    }

    protected void setMotorPowers(double vFL, double vFR, double vBL, double vBR) {
        fLeft.setPower(vFL);
        fRight.setPower(vFR);
        bLeft.setPower(vBL);
        bRight.setPower(vBR);
    }

}
