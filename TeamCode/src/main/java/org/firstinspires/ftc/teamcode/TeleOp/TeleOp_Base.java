package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.enforcers7149.touchpadplusplus.src.Touchpad;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Odometry.DriveWheels.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.EnforcersTouchpad;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors.BulkRead;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Input;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Output;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors.Gyroscope;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.VelLimitsJerk;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.GlobalData.*;
import static org.firstinspires.ftc.teamcode.Subsystems.Utils.FixedRoadrunner.createPose2d;

import android.util.Log;

public abstract class TeleOp_Base extends OpMode {

    //Drive
    protected MecanumDrive drive;
    protected DcMotorEx fLeft, fRight, bLeft, bRight;
    protected Gyroscope gyro;
    private boolean initializedMotors = false, initializedDrive = false, initializedGyro = false;

    //Touchpad
    protected EnforcersTouchpad touchpad1, touchpad2;

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
    private boolean initializedSources = false;

    //Initialization
    protected void initializeSources() {
        inputs = new ArrayList<Input>();
        outputs = new ArrayList<Output>();
        initializedSources = true;
    }
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
        if (!initializedSources) initializeSources();

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
        if (!initializedSources) initializeSources();

        if (!initializedDrive)
            gyro = new Gyroscope(hardwareMap);
        else
            gyro = drive.gyro;

        inputs.add((hasCH?1:0) + (hasEH?1:0), gyro);
        initializedGyro = true;
    }
    protected void initializeOdometry() throws Exception {
        if (!initializedSources) initializeSources();

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

        fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

    /*protected void initializeTouchpad() {

        touchpad1 = new EnforcersTouchpad(gamepad1);
        touchpad2 = new EnforcersTouchpad(gamepad2);

        addInput(touchpad1);
        addInput(touchpad2);

    }*/

    protected void initializeAll() throws Exception {
        initializeSources();
        initializeDrive();
        initializeBulkRead();
        initializeGyro();
        initializeOdometry();
        initializeVars();
        //initializeTouchpad();
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
        logData();
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

    protected void logData() {

        Log.i("\nTime ", String.valueOf(System.currentTimeMillis()) + '\n');

        Log.i("Pose ", drive.getPoseEstimate().toString() + '\n');

        Log.i("X Accel ", String.valueOf(gyro.imu.getAcceleration().xAccel));
        Log.i("Y Accel ", String.valueOf(gyro.imu.getAcceleration().yAccel));
        Log.i("Z Accel ", String.valueOf(gyro.imu.getAcceleration().zAccel) + '\n');

        Log.i("fL Power ", String.valueOf(fLeft.getPower()));
        Log.i("fR Power ", String.valueOf(fRight.getPower()));
        Log.i("bL Power ", String.valueOf(bLeft.getPower()));
        Log.i("bR Power ", String.valueOf(bRight.getPower()) + '\n');

        // in ticks/s
        Log.i("fL Velocity (Motor) ", String.valueOf(fLeft.getVelocity()));
        Log.i("fR Velocity (Motor) ", String.valueOf(fRight.getVelocity()));
        Log.i("bL Velocity (Motor) ", String.valueOf(bLeft.getVelocity()));
        Log.i("bR Velocity (Motor) ", String.valueOf(bRight.getVelocity()) + '\n');

        Log.i("fL Velocity (Odo) ", String.valueOf(drive.getWheelVelocities().get(0)));
        Log.i("fR Velocity (Odo) ", String.valueOf(drive.getWheelVelocities().get(1)));
        Log.i("bL Velocity (Odo) ", String.valueOf(drive.getWheelVelocities().get(2)));
        Log.i("bR Velocity (Odo) ", String.valueOf(drive.getWheelVelocities().get(3)));
        Log.i("Overall velocity (Odo) ", drive.getPoseVelocity().toString() + '\n');


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
            drive.setPoseEstimate(new Pose2d(
                    drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), 0));
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
            drive.setPoseEstimate(new Pose2d(
                    drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), 0));
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

    //Automated driving
    private enum DriveState {
        DRIVE, SHARED_BARRIER
    }
    private DriveState currDriveState = DriveState.DRIVE, lastDriveState = DriveState.DRIVE;
    private Pose2d driveStartPose = new Pose2d(0, 0, 0);
    private PIDFController hControl;

    protected final void startSharedBarrierForward() {
        currDriveState = DriveState.SHARED_BARRIER;
    }
    protected final void updateAutomatedDriving() {
        if (currDriveState != lastDriveState) {
            driveStartPose = drive.getPoseEstimate();
            hControl = new PIDFController(H_PID);
            hControl.setOutputBounds(0, 1);
            hControl.setTargetPosition(0);
        }

        boolean driveCompleted = true;

        switch (currDriveState) {
            case SHARED_BARRIER:
                driveCompleted = driveTo(driveStartPose.plus(new Pose2d(24, 0, 0)));
                break;
            case DRIVE:
            default:
                break;
        }

        lastDriveState = currDriveState;

        if (driveCompleted) currDriveState = DriveState.DRIVE;
    }
    protected final void cancelAutomatedDriving() {
        currDriveState = DriveState.DRIVE;
    }
    protected final boolean isAutomatedDriving() {
        return currDriveState != DriveState.DRIVE;
    }


    /**
     * Sets drive powers to move from one point to another
     * @param endPose Destination position
     * @return False while driving, true if done
     */
    public boolean driveTo(Pose2d endPose) {
        double destX = endPose.getX();
        double destY = endPose.getY();
        double destH = endPose.getHeading();

        //Current robot position
        double robotX = drive.getPoseEstimate().getX();
        double robotY = drive.getPoseEstimate().getY();
        double robotH = drive.getPoseEstimate().getHeading();

        //Calculate relatives
        double relX = destX - robotX;
        double relY = destY - robotY;
        double relH = deltaHeading(robotH, destH);

        double hWeight;

        //While robot is not at the current destination point
        if ((Math.abs(relX) > 2 || //POSE_ACC
                        Math.abs(relY) > 2 || //POSE_ACC
                        Math.abs(relH) > H_ACC)) {

            //Update robot position
            robotX = drive.getPoseEstimate().getX();
            robotY = drive.getPoseEstimate().getY();
            robotH = drive.getPoseEstimate().getHeading();

            //Calculate relatives
            relX = destX - robotX;
            relY = destY - robotY;
            relH = deltaHeading(robotH, destH);

            hWeight = hControl.update(Math.abs(relH));

            if (Math.abs(relX) < 2) relX = 0; //POSE_ACC
            if (Math.abs(relY) < 2) relY = 0; //POSE_ACC
            if (Math.abs(relH) < H_ACC) hWeight = 0;

            double driveAngle = deltaHeading(robotH, Math.atan2(relY, relX));

            double xPower = Math.cos(driveAngle);
            double yPower = Math.sin(driveAngle);
            double hPower = Math.copySign(hWeight, relH);

            double dist = Math.sqrt((relX*relX) + (relY*relY));

            if (dist <= 5) { //SLOW_DIST
                xPower *= Math.pow(dist / 5, 3);
                yPower *= Math.pow(dist / 5, 3);
            }

            double max = Math.max(Math.abs(xPower), Math.abs(yPower));

            if (max < 0.2 && max != 0) { //MIN_SPEED
                xPower /= max / 0.2;
                yPower /= max / 0.2;
            }
            if (Math.abs(hPower) < 0.2 && Math.abs(hPower) > 0) //MIN_TURN
                hPower = Math.copySign(0.2, hPower);

            drive.setWeightedDrivePower(new Pose2d(xPower, yPower, hPower));

            return false;
        }
        else
            setMotorPowers(0, 0, 0, 0);

        return true;
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
     * @param robotH Robot heading in radians
     * @param destH Destination heading in radians
     * @return Shortest heading difference in radians
     */
    protected double deltaHeading(double robotH, double destH) {
        if (robotH < 0) robotH += Math.PI * 2;
        if (destH < 0) destH += Math.PI * 2;

        double diff = destH - robotH;

        if (diff < -Math.PI) diff += Math.PI * 2;
        if (diff > Math.PI) diff -= Math.PI * 2;

        return diff;
    }

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
