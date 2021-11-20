package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.teamcode.Odometry.DriveWheels.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.BulkRead;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors.Gyroscope;

import static org.firstinspires.ftc.teamcode.Subsystems.FixedRoadrunner.createPose2d;

@Config
public abstract class Autonomous_Base extends LinearOpMode {

    //Drive
    protected MecanumDrive drive;
    public static PIDCoefficients H_PID = new PIDCoefficients(-0.5, 0, -0.05);
    protected DcMotorEx fLeft, fRight, bLeft, bRight;
    protected Gyroscope gyro;
    private boolean initializedMotors = false, initializedDrive = false, initializedGyro = false;

    //Control objects
    protected BulkRead bReadCH, bReadEH;
    private boolean hasCH, hasEH;
    protected AutoCommands commands;

    protected boolean USE_SUBS;

    //How accurate each attribute should be at each point
    public static double POS_ACC = 0.1;
    public static double H_ACC = Math.toRadians(1);

    public static double MIN_SPEED = 0.2;
    public static double MIN_TURN = 0.1;
    public static double CLOSE_DIST = 0;

    public static double SLOW_DIST = 15;

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
    protected double deltaHeading(double robotH, double destH) {
        double diff = destH - robotH;

        if (diff < -Math.PI) diff += Math.PI * 2;
        if (diff > Math.PI) diff -= Math.PI * 2;

        return diff;
    }

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
        if (!hasEH) throw new Exception("Missing \"Expansion Hub\". Check configuration file naming");
        if (initializedMotors && initializedGyro)
            drive = new MecanumDrive(hardwareMap, bReadCH, bReadEH, fLeft, fRight, bLeft, bRight, gyro);
        else if (initializedMotors)
            drive = new MecanumDrive(hardwareMap, bReadCH, bReadEH, fLeft, fRight, bLeft, bRight);
        else if (initializedGyro)
            drive = new MecanumDrive(hardwareMap, bReadCH, bReadEH, gyro);
        else
            drive = new MecanumDrive(hardwareMap, bReadCH, bReadEH);
        drive.setPoseEstimate(createPose2d(0, 0, 0));

        initializedDrive = true;
    }
    protected void initializeCommands() {
        commands = new AutoCommands(this);
    }
    protected void initializeAll() throws Exception {
        initializeDrive();
        initializeBulkRead();
        initializeGyro();
        initializeOdometry();
        initializeCommands();
    }

    //Loop updates
    protected void updateBulkRead() {
        if (hasCH) bReadCH.update();
        if (hasEH) bReadEH.update();
    }

    protected abstract Alliance getAlliance();

    protected abstract void subsystemUpdates();
    protected abstract void addTelemetryData();

    protected final void updateSubsystems() {
        if (USE_SUBS) subsystemUpdates();
    }
    protected final void updateTelemetry() {
        addTelemetryData();
        telemetry.update();
    }

    //Useful functions
    protected void updateInputs() {
        updateBulkRead();
        if (initializedGyro) gyro.update();
        if (initializedDrive) drive.update();
    }

    protected void updateOutputs() {
        updateSubsystems();
        updateTelemetry();
    }

    protected void customWait(Func<Boolean> func) {
        while (opModeIsActive() && func.value()) {
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
}
