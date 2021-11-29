package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Odometry.DriveWheels.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.BulkRead;
import org.firstinspires.ftc.teamcode.Subsystems.Input;
import org.firstinspires.ftc.teamcode.Subsystems.Output;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors.Gyroscope;
import org.firstinspires.ftc.teamcode.Subsystems.VelLimitsJerk;

import java.util.ArrayList;

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

    //Inputs & Outputs
    ArrayList<Input> inputs;
    ArrayList<Output> outputs;

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
            bReadEH = new BulkRead(hardwareMap, "Expansion Hub");
            inputs.add(0, bReadEH);
            hasEH = true;
        } catch (Exception e) {
            hasEH = false;
        }
        try {
            bReadCH = new BulkRead(hardwareMap, "Control Hub");
            inputs.add(0, bReadCH);
            hasCH = true;
        } catch (Exception e) {
            hasCH = false;
        }
    }
    protected void initializeGyro() {
        if (!initializedDrive)
            gyro = new Gyroscope(hardwareMap);
        else
            gyro = drive.gyro;

        inputs.add((hasCH?1:0) + (hasEH?1:0), gyro);
        initializedGyro = true;
    }
    protected void initializeOdometry() throws Exception {
        if (!hasCH || !hasEH) throw new Exception("Missing \"Control Hub\". Check configuration file naming");
        if (initializedMotors && initializedGyro)
            drive = new MecanumDrive(hardwareMap, bReadCH, bReadEH, fLeft, fRight, bLeft, bRight, gyro);
        else if (initializedMotors)
            drive = new MecanumDrive(hardwareMap, bReadCH, bReadEH, fLeft, fRight, bLeft, bRight);
        else if (initializedGyro)
            drive = new MecanumDrive(hardwareMap, bReadCH, bReadEH, gyro);
        else
            drive = new MecanumDrive(hardwareMap, bReadCH, bReadEH);
        drive.setPoseEstimate(createPose2d(0, 0, 0));

        inputs.add((hasCH?1:0) + (hasEH?1:0) + (initializedGyro?1:0), drive);
        outputs.add(0, drive);
        initializedDrive = true;
    }
    protected void initializeVars() {
        lastLeftX = 0; lastLeftY = 0; lastRightX = 0; lastRightY = 0;
        offset = 0; lim = 1;

        yJerk = new VelLimitsJerk(300);
        xJerk = new VelLimitsJerk(0);
        turnJerk = new VelLimitsJerk(0);
    }
    protected void initializeAll() throws Exception {
        initializeDrive();
        initializeBulkRead();
        initializeGyro();
        initializeOdometry();
        initializeVars();
    }

    protected void addInput(Input input) {
        inputs.add(input);
    }
    protected void addOutput(Output output) {
        outputs.add(output);
    }

    //Start
    protected void startInputs() {
        for (Input i : inputs) i.startInput();
    }
    protected void startOutputs() {
        for (Output o : outputs) o.startOutput();
    }

    //Loop
    protected void updateInputs() {
        for (Input i : inputs) i.updateInput();
    }
    protected void updateOutputs() {
        for (Output o : outputs) o.updateOutput();
    }

    //Stop
    protected void stopInputs() {
        for (Input i : inputs) i.stopInput();
    }
    protected void stopOutputs() {
        for (Output o : outputs) o.stopOutput();
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

    //Smooth driving
    private VelLimitsJerk yJerk, xJerk, turnJerk;

    protected final void driveHeadlessSmooth(double angle, boolean reset) {
        if (reset) {
            offset = angle;
        }

        if ((leftY == 0 && lastLeftY != 0) &&
                (leftX == 0 && lastLeftX != 0) &&
                (rightX == 0 && lastRightX != 0)) {
            setMotorPowers(0, 0, 0, 0);
            return;
        }

        double r = Math.hypot(leftX, leftY);
        double robotAngle = Math.atan2(leftY, leftX) - Math.toRadians(Gyroscope.cvtTrigAng(angle - offset)) - 3 * Math.PI / 4;

        double vFL = r * Math.sin(robotAngle) + rightX;
        double vFR = r * Math.cos(robotAngle) - rightX;
        double vBL = r * Math.cos(robotAngle) + rightX;
        double vBR = r * Math.sin(robotAngle) - rightX;

        //Get robot relative components of motion
        double robotY = (vFL + vFR + vBL + vBR) / 4;
        double robotX = (vFL - vFR - vBL + vBR) / 4;
        double robotTurn = (vFL - vFR + vBL - vBR) / 4;

        //Limit the jerk of each component of motion
        robotY = yJerk.update(robotY);
        robotX = xJerk.update(robotX);
        robotTurn = turnJerk.update(robotTurn);

        double limFixed = lim / (2 - lim);
        double mult = limFixed / (limFixed + Math.abs(robotTurn));

        //Convert back to motor powers
        vFL = mult * (robotY + robotX - robotTurn);
        vFR = mult * (robotY - robotX + robotTurn);
        vBL = mult * (robotY - robotX - robotTurn);
        vBR = mult * (robotY + robotX + robotTurn);

        setMotorPowers(vFL, vFR, vBL, vBR);
    }

    /**
     * Sets the new "maxTime" for velocity curves
     * @param yTime yJerk time in ms
     * @param xTime xJerk time in ms
     * @param tTime turnJerk time in ms
     */
    protected final void setSmoothingTimes(double yTime, double xTime, double tTime) {
        yJerk.setMaxTime(yTime);
        xJerk.setMaxTime(xTime);
        turnJerk.setMaxTime(tTime);
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
