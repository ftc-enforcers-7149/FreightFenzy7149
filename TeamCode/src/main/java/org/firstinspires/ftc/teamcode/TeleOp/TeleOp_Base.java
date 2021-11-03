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
