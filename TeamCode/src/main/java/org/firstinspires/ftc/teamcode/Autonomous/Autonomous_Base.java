package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Odometry.DriveWheels.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors.BulkRead;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Input;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Output;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors.Gyroscope;

import java.util.ArrayList;
import java.util.function.Supplier;

import static org.firstinspires.ftc.teamcode.GlobalData.*;
import static org.firstinspires.ftc.teamcode.Subsystems.Utils.FixedRoadrunner.createPose2d;

import android.util.Log;

@Config
public abstract class Autonomous_Base extends LinearOpMode {

    //Drive
    protected MecanumDrive drive;
    protected DcMotorEx fLeft, fRight, bLeft, bRight;
    protected Gyroscope gyro;
    private boolean initializedMotors = false, initializedDrive = false, initializedGyro = false;

    //Control objects
    protected BulkRead bReadCH, bReadEH;
    private boolean hasCH, hasEH;
    protected AutoCommands commands;

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

        inputs.add((hasCH?1:0) + (hasEH?1:0) + (initializedGyro?1:0), drive);
        outputs.add(0, drive);
        initializedDrive = true;
    }
    protected void initializeCommands() {
        commands = new AutoCommands(this);
    }

    protected void initializeAll() throws Exception {
        initializeSources();
        initializeDrive();
        initializeBulkRead();
        initializeGyro();
        initializeOdometry();
        initializeCommands();
    }

    protected void addInput(Input input) {
        inputs.add(input);
    }
    protected void addOutput(Output output) {
        outputs.add(output);
    }

    protected void removeOutput(Output output) {
        outputs.remove(output);
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
        for (Output o : actionQueue) {
            addOutput(o);
            o.startOutput();
        }

        for (Output o : outputs) o.updateOutput();
        updateTelemetry();
    }

    //Stop
    protected void stopInputs() {
        for (Input i : inputs) i.stopInput();
    }
    protected void stopOutputs() {
        for (Output o : outputs) o.stopOutput();
    }

    public void driveTo(double destX, double destY, double destH) {
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PIDFController hControl = new PIDFController(H_PID);
        hControl.setOutputBounds(0, 1);
        hControl.setTargetPosition(0);

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
        while (opModeIsActive() &&
                (Math.abs(relX) > POS_ACC ||
                        Math.abs(relY) > POS_ACC ||
                        Math.abs(relH) > H_ACC)) {

            updateInputs();

            //Update robot position
            robotX = drive.getPoseEstimate().getX();
            robotY = drive.getPoseEstimate().getY();
            robotH = drive.getPoseEstimate().getHeading();

            //Calculate relatives
            relX = destX - robotX;
            relY = destY - robotY;
            relH = deltaHeading(robotH, destH);

            hWeight = hControl.update(Math.abs(relH));

            if (Math.abs(relX) < POS_ACC) relX = 0;
            if (Math.abs(relY) < POS_ACC) relY = 0;
            if (Math.abs(relH) < H_ACC) hWeight = 0;

            double driveAngle = deltaHeading(robotH, Math.atan2(relY, relX));

            double xPower = Math.cos(driveAngle) * SPEED_MULT;
            double yPower = Math.sin(driveAngle) * SPEED_MULT;
            double hPower = Math.copySign(hWeight, relH) * SPEED_MULT;

            double dist = Math.sqrt((relX*relX) + (relY*relY));

            if (dist <= SLOW_DIST) {
                xPower *= Math.pow(dist / SLOW_DIST, 2);
                yPower *= Math.pow(dist / SLOW_DIST, 2);
            }

            double max = Math.max(Math.abs(xPower), Math.abs(yPower));
            if (dist <= CLOSE_DIST && max != 0) {
                xPower /= max / MIN_SPEED;
                yPower /= max / MIN_SPEED;
            }
            else {
                if (max < MIN_SPEED && max != 0) {
                    xPower /= max / MIN_SPEED;
                    yPower /= max / MIN_SPEED;
                }
                if (Math.abs(hPower) < MIN_TURN && Math.abs(hPower) > 0)
                    hPower = Math.copySign(MIN_TURN, hPower);
            }

            drive.setWeightedDrivePower(new Pose2d(xPower, yPower, hPower));

            telemetry.addData("X Power: ", xPower);
            telemetry.addData("Y Power: ", yPower);
            telemetry.addData("H Power: ", hPower);

            updateOutputs();
        }

        setMotorPowers(0, 0, 0, 0);
    }
    public void driveTo(double destX, double destY, double destH, boolean stop) {
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PIDFController hControl = new PIDFController(H_PID);
        hControl.setOutputBounds(0, 1);
        hControl.setTargetPosition(0);

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
        while (opModeIsActive() &&
                (Math.abs(relX) > POS_ACC ||
                        Math.abs(relY) > POS_ACC ||
                        Math.abs(relH) > H_ACC)) {

            updateInputs();

            //Update robot position
            robotX = drive.getPoseEstimate().getX();
            robotY = drive.getPoseEstimate().getY();
            robotH = drive.getPoseEstimate().getHeading();

            //Calculate relatives
            relX = destX - robotX;
            relY = destY - robotY;
            relH = deltaHeading(robotH, destH);

            hWeight = hControl.update(Math.abs(relH));

            if (Math.abs(relX) < POS_ACC) relX = 0;
            if (Math.abs(relY) < POS_ACC) relY = 0;
            if (Math.abs(relH) < H_ACC) hWeight = 0;

            double driveAngle = deltaHeading(robotH, Math.atan2(relY, relX));

            double xPower = Math.cos(driveAngle);
            double yPower = Math.sin(driveAngle);
            double hPower = Math.copySign(hWeight, relH);

            double dist = Math.sqrt((relX*relX) + (relY*relY));

            if (dist <= (stop ? SLOW_DIST : -1)) {
                xPower *= Math.pow(dist / SLOW_DIST, 2);
                yPower *= Math.pow(dist / SLOW_DIST, 2);
            }

            double max = Math.max(Math.abs(xPower), Math.abs(yPower));
            if (dist <= (stop ? CLOSE_DIST : -1) && max != 0) {
                xPower /= max / MIN_SPEED;
                yPower /= max / MIN_SPEED;
            }
            else {
                if (max < MIN_SPEED && max != 0) {
                    xPower /= max / MIN_SPEED;
                    yPower /= max / MIN_SPEED;
                }
                if (Math.abs(hPower) < MIN_TURN && Math.abs(hPower) > 0)
                    hPower = Math.copySign(MIN_TURN, hPower);
            }

            drive.setWeightedDrivePower(new Pose2d(xPower, yPower, hPower));

            telemetry.addData("X Power: ", xPower);
            telemetry.addData("Y Power: ", yPower);
            telemetry.addData("H Power: ", hPower);

            updateOutputs();
        }

        if (stop) setMotorPowers(0, 0, 0, 0);
    }
    public void driveTo(Supplier<Double> destX, Supplier<Double> destY, Supplier<Double> destH) {
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PIDFController hControl = new PIDFController(H_PID);
        hControl.setOutputBounds(0, 1);
        hControl.setTargetPosition(0);

        //Current robot position
        double robotX = drive.getPoseEstimate().getX();
        double robotY = drive.getPoseEstimate().getY();
        double robotH = drive.getPoseEstimate().getHeading();

        //Calculate relatives
        double relX = destX.get() - robotX;
        double relY = destY.get() - robotY;
        double relH = deltaHeading(robotH, destH.get());

        double hWeight;

        //While robot is not at the current destination point
        while (opModeIsActive() &&
                (Math.abs(relX) > POS_ACC ||
                        Math.abs(relY) > POS_ACC ||
                        Math.abs(relH) > H_ACC)) {

            updateInputs();

            //Update robot position
            robotX = drive.getPoseEstimate().getX();
            robotY = drive.getPoseEstimate().getY();
            robotH = drive.getPoseEstimate().getHeading();

            //Calculate relatives
            relX = destX.get() - robotX;
            relY = destY.get() - robotY;
            relH = deltaHeading(robotH, destH.get());

            hWeight = hControl.update(Math.abs(relH));

            if (Math.abs(relX) < POS_ACC) relX = 0;
            if (Math.abs(relY) < POS_ACC) relY = 0;
            if (Math.abs(relH) < H_ACC) hWeight = 0;

            double driveAngle = deltaHeading(robotH, Math.atan2(relY, relX));

            double xPower = Math.cos(driveAngle);
            double yPower = Math.sin(driveAngle);
            double hPower = Math.copySign(hWeight, relH);

            double dist = Math.sqrt((relX*relX) + (relY*relY));

            if (dist <= SLOW_DIST) {
                xPower *= Math.pow(dist / SLOW_DIST, 2);
                yPower *= Math.pow(dist / SLOW_DIST, 2);
            }

            double max = Math.max(Math.abs(xPower), Math.abs(yPower));
            if (dist <= CLOSE_DIST && max != 0) {
                xPower /= max / MIN_SPEED;
                yPower /= max / MIN_SPEED;
            }
            else {
                if (max < MIN_SPEED && max != 0) {
                    xPower /= max / MIN_SPEED;
                    yPower /= max / MIN_SPEED;
                }
                if (hPower != 0 && Math.abs(hPower) < MIN_TURN)
                    hPower = Math.copySign(MIN_TURN, hPower);
            }

            drive.setWeightedDrivePower(new Pose2d(xPower, yPower, hPower));

            telemetry.addData("X Power: ", xPower);
            telemetry.addData("Y Power: ", yPower);
            telemetry.addData("H Power: ", hPower);

            updateOutputs();
        }

        setMotorPowers(0, 0, 0, 0);
    }
    public boolean driveTo(Supplier<Double> destX, Supplier<Double> destY, Supplier<Double> destH,
                        long timeOut) {
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PIDFController hControl = new PIDFController(H_PID);
        hControl.setOutputBounds(0, 1);
        hControl.setTargetPosition(0);

        //Current robot position
        double robotX = drive.getPoseEstimate().getX();
        double robotY = drive.getPoseEstimate().getY();
        double robotH = drive.getPoseEstimate().getHeading();

        //Calculate relatives
        double relX = destX.get() - robotX;
        double relY = destY.get() - robotY;
        double relH = deltaHeading(robotH, destH.get());

        double hWeight;

        long startTime = System.currentTimeMillis();

        //While robot is not at the current destination point
        while (opModeIsActive() && System.currentTimeMillis() < startTime + timeOut &&
                (Math.abs(relX) > POS_ACC ||
                        Math.abs(relY) > POS_ACC ||
                        Math.abs(relH) > H_ACC)) {

            updateInputs();

            //Update robot position
            robotX = drive.getPoseEstimate().getX();
            robotY = drive.getPoseEstimate().getY();
            robotH = drive.getPoseEstimate().getHeading();

            //Calculate relatives
            relX = destX.get() - robotX;
            relY = destY.get() - robotY;
            relH = deltaHeading(robotH, destH.get());

            hWeight = hControl.update(Math.abs(relH));

            if (Math.abs(relX) < POS_ACC) relX = 0;
            if (Math.abs(relY) < POS_ACC) relY = 0;
            if (Math.abs(relH) < H_ACC) hWeight = 0;

            double driveAngle = deltaHeading(robotH, Math.atan2(relY, relX));

            double xPower = Math.cos(driveAngle);
            double yPower = Math.sin(driveAngle);
            double hPower = Math.copySign(hWeight, relH);

            double dist = Math.sqrt((relX*relX) + (relY*relY));

            if (dist <= SLOW_DIST) {
                xPower *= Math.pow(dist / SLOW_DIST, 2);
                yPower *= Math.pow(dist / SLOW_DIST, 2);
            }

            double max = Math.max(Math.abs(xPower), Math.abs(yPower));
            if (dist <= CLOSE_DIST && max != 0) {
                xPower /= max / MIN_SPEED;
                yPower /= max / MIN_SPEED;
            }
            else {
                if (max < MIN_SPEED && max != 0) {
                    xPower /= max / MIN_SPEED;
                    yPower /= max / MIN_SPEED;
                }
                if (hPower != 0 && Math.abs(hPower) < MIN_TURN)
                    hPower = Math.copySign(MIN_TURN, hPower);
            }

            drive.setWeightedDrivePower(new Pose2d(xPower, yPower, hPower));

            telemetry.addData("X Power: ", xPower);
            telemetry.addData("Y Power: ", yPower);
            telemetry.addData("H Power: ", hPower);

            updateOutputs();
        }

        setMotorPowers(0, 0, 0, 0);

        return System.currentTimeMillis() < startTime + timeOut;
    }
    public void driveTo(Supplier<Double> destX, Supplier<Double> destY, Supplier<Double> destH,
                        boolean stop) {
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PIDFController hControl = new PIDFController(H_PID);
        hControl.setOutputBounds(0, 1);
        hControl.setTargetPosition(0);

        //Current robot position
        double robotX = drive.getPoseEstimate().getX();
        double robotY = drive.getPoseEstimate().getY();
        double robotH = drive.getPoseEstimate().getHeading();

        //Calculate relatives
        double relX = destX.get() - robotX;
        double relY = destY.get() - robotY;
        double relH = deltaHeading(robotH, destH.get());

        double hWeight;

        //While robot is not at the current destination point
        while (opModeIsActive() &&
                (Math.abs(relX) > POS_ACC ||
                        Math.abs(relY) > POS_ACC ||
                        Math.abs(relH) > H_ACC)) {

            updateInputs();

            //Update robot position
            robotX = drive.getPoseEstimate().getX();
            robotY = drive.getPoseEstimate().getY();
            robotH = drive.getPoseEstimate().getHeading();

            //Calculate relatives
            relX = destX.get() - robotX;
            relY = destY.get() - robotY;
            relH = deltaHeading(robotH, destH.get());

            hWeight = hControl.update(Math.abs(relH));

            if (Math.abs(relX) < POS_ACC) relX = 0;
            if (Math.abs(relY) < POS_ACC) relY = 0;
            if (Math.abs(relH) < H_ACC) hWeight = 0;

            double driveAngle = deltaHeading(robotH, Math.atan2(relY, relX));

            double xPower = Math.cos(driveAngle);
            double yPower = Math.sin(driveAngle);
            double hPower = Math.copySign(hWeight, relH);

            double dist = Math.sqrt((relX*relX) + (relY*relY));

            if (dist <= (stop ? SLOW_DIST : -1)) {
                xPower *= Math.pow(dist / SLOW_DIST, 2);
                yPower *= Math.pow(dist / SLOW_DIST, 2);
            }

            double max = Math.max(Math.abs(xPower), Math.abs(yPower));
            if (dist <= (stop ? CLOSE_DIST : -1) && max != 0) {
                xPower /= max / MIN_SPEED;
                yPower /= max / MIN_SPEED;
            }
            else {
                if (max < MIN_SPEED && max != 0) {
                    xPower /= max / MIN_SPEED;
                    yPower /= max / MIN_SPEED;
                }
                if (Math.abs(hPower) < MIN_TURN && Math.abs(hPower) > 0)
                    hPower = Math.copySign(MIN_TURN, hPower);
            }

            drive.setWeightedDrivePower(new Pose2d(xPower, yPower, hPower));

            telemetry.addData("X Power: ", xPower);
            telemetry.addData("Y Power: ", yPower);
            telemetry.addData("H Power: ", hPower);

            updateOutputs();
        }

        if (stop) setMotorPowers(0, 0, 0, 0);
    }
    //Returns TRUE when times out
    public boolean driveTo(Supplier<Double> destX, Supplier<Double> destY, Supplier<Double> destH,
                        long timeOut, boolean stop) {
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PIDFController hControl = new PIDFController(H_PID);
        hControl.setOutputBounds(0, 1);
        hControl.setTargetPosition(0);

        //Current robot position
        double robotX = drive.getPoseEstimate().getX();
        double robotY = drive.getPoseEstimate().getY();
        double robotH = drive.getPoseEstimate().getHeading();

        //Calculate relatives
        double relX = destX.get() - robotX;
        double relY = destY.get() - robotY;
        double relH = deltaHeading(robotH, destH.get());

        double hWeight;

        long startTime = System.currentTimeMillis();

        //While robot is not at the current destination point
        while (opModeIsActive() && System.currentTimeMillis() < startTime + timeOut &&
                (Math.abs(relX) > POS_ACC ||
                        Math.abs(relY) > POS_ACC ||
                        Math.abs(relH) > H_ACC)) {

            updateInputs();

            //Update robot position
            robotX = drive.getPoseEstimate().getX();
            robotY = drive.getPoseEstimate().getY();
            robotH = drive.getPoseEstimate().getHeading();

            //Calculate relatives
            relX = destX.get() - robotX;
            relY = destY.get() - robotY;
            relH = deltaHeading(robotH, destH.get());

            hWeight = hControl.update(Math.abs(relH));

            if (Math.abs(relX) < POS_ACC) relX = 0;
            if (Math.abs(relY) < POS_ACC) relY = 0;
            if (Math.abs(relH) < H_ACC) hWeight = 0;

            double driveAngle = deltaHeading(robotH, Math.atan2(relY, relX));

            double xPower = Math.cos(driveAngle);
            double yPower = Math.sin(driveAngle);
            double hPower = Math.copySign(hWeight, relH);

            double dist = Math.sqrt((relX*relX) + (relY*relY));

            if (dist <= (stop ? SLOW_DIST : -1)) {
                xPower *= Math.pow(dist / SLOW_DIST, 2);
                yPower *= Math.pow(dist / SLOW_DIST, 2);
            }

            double max = Math.max(Math.abs(xPower), Math.abs(yPower));
            if (dist <= (stop ? CLOSE_DIST : -1) && max != 0) {
                xPower /= max / MIN_SPEED;
                yPower /= max / MIN_SPEED;
            }
            else {
                if (max < MIN_SPEED && max != 0) {
                    xPower /= max / MIN_SPEED;
                    yPower /= max / MIN_SPEED;
                }
                if (Math.abs(hPower) < MIN_TURN && Math.abs(hPower) > 0)
                    hPower = Math.copySign(MIN_TURN, hPower);
            }

            drive.setWeightedDrivePower(new Pose2d(xPower, yPower, hPower));

            telemetry.addData("X Power: ", xPower);
            telemetry.addData("Y Power: ", yPower);
            telemetry.addData("H Power: ", hPower);

            updateOutputs();
        }

        if (stop) setMotorPowers(0, 0, 0, 0);

        return System.currentTimeMillis() >= startTime + timeOut;
    }
    public void driveTo(double destX, double destY, double destH, long timeOut) {
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PIDFController hControl = new PIDFController(H_PID);
        hControl.setOutputBounds(0, 1);
        hControl.setTargetPosition(0);

        //Current robot position
        double robotX = drive.getPoseEstimate().getX();
        double robotY = drive.getPoseEstimate().getY();
        double robotH = drive.getPoseEstimate().getHeading();

        //Calculate relatives
        double relX = destX - robotX;
        double relY = destY - robotY;
        double relH = deltaHeading(robotH, destH);

        double hWeight;

        long startTime = System.currentTimeMillis();

        //While robot is not at the current destination point
        while (opModeIsActive() && System.currentTimeMillis() < startTime + timeOut &&
                (Math.abs(relX) > POS_ACC ||
                        Math.abs(relY) > POS_ACC ||
                        Math.abs(relH) > H_ACC)) {

            updateInputs();

            //Update robot position
            robotX = drive.getPoseEstimate().getX();
            robotY = drive.getPoseEstimate().getY();
            robotH = drive.getPoseEstimate().getHeading();

            //Calculate relatives
            relX = destX - robotX;
            relY = destY - robotY;
            relH = deltaHeading(robotH, destH);

            hWeight = hControl.update(Math.abs(relH));

            if (Math.abs(relX) < POS_ACC) relX = 0;
            if (Math.abs(relY) < POS_ACC) relY = 0;
            if (Math.abs(relH) < H_ACC) hWeight = 0;

            double driveAngle = deltaHeading(robotH, Math.atan2(relY, relX));

            double xPower = Math.cos(driveAngle) * SPEED_MULT;
            double yPower = Math.sin(driveAngle) * SPEED_MULT;
            double hPower = Math.copySign(hWeight, relH) * SPEED_MULT;

            double dist = Math.sqrt((relX*relX) + (relY*relY));

            if (dist <= SLOW_DIST) {
                xPower *= Math.pow(dist / SLOW_DIST, 2);
                yPower *= Math.pow(dist / SLOW_DIST, 2);
            }

            double max = Math.max(Math.abs(xPower), Math.abs(yPower));
            if (dist <= CLOSE_DIST && max != 0) {
                xPower /= max / MIN_SPEED;
                yPower /= max / MIN_SPEED;
            }
            else {
                if (max < MIN_SPEED && max != 0) {
                    xPower /= max / MIN_SPEED;
                    yPower /= max / MIN_SPEED;
                }
                if (Math.abs(hPower) < MIN_TURN && Math.abs(hPower) > 0)
                    hPower = Math.copySign(MIN_TURN, hPower);
            }

            drive.setWeightedDrivePower(new Pose2d(xPower, yPower, hPower));

            telemetry.addData("X Power: ", xPower);
            telemetry.addData("Y Power: ", yPower);
            telemetry.addData("H Power: ", hPower);

            updateOutputs();
        }

        setMotorPowers(0, 0, 0, 0);
    }

    /**
     * Moves to a specific position without using Roadrunner trajectories
     * @param destX The destination x position
     * @param destY The destination y position
     */
    public void moveTo(double destX, double destY) {
        driveTo(destX, destY, drive.getPoseEstimate().getHeading());
    }

    /**
     * Rotates to a specific angle, without using Roadrunner trajectories
     * @param destAngle The destination angle in Radians
     */
    public void rotateTo(double destAngle) {
        driveTo(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), destAngle);
    }

    /**
     * @param robotH Robot heading in radians
     * @param destH Destination heading in radians
     * @return Shortest heading difference in radians
     */
    public static double deltaHeading(double robotH, double destH) {
        while (robotH < 0) robotH += Math.PI * 2;
        while (destH < 0) destH += Math.PI * 2;

        double diff = destH - robotH;

        while (diff < -Math.PI) diff += Math.PI * 2;
        while (diff > Math.PI) diff -= Math.PI * 2;

        return diff;
    }

    //Useful Functions
    protected abstract Alliance getAlliance();
    protected abstract void addTelemetryData();
    protected final void updateTelemetry() {
        addTelemetryData();
        telemetry.update();
    }

    protected void customWait(Supplier<Boolean> value) {
        while (opModeIsActive() && value.get()) {
            updateInputs();
            updateOutputs();
        }
    }

    protected void waitForDriveComplete() {
        while (opModeIsActive() && drive.isBusy()) {
            updateInputs();
            updateOutputs();
        }
    }

    protected void waitForTime(double ms) {
        double startTime = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() < startTime + ms) {
            updateInputs();
            updateOutputs();
        }
    }

    protected void setMotorPowers(double v1, double v2, double v3, double v4) {
        fLeft.setPower(v1);
        fRight.setPower(v2);
        bLeft.setPower(v3);
        bRight.setPower(v4);
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

}
