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
import org.firstinspires.ftc.teamcode.Subsystems.BulkRead;
import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;

import static org.firstinspires.ftc.teamcode.Subsystems.FixedRoadrunner.createPose2d;

@Config
public abstract class Autonomous_Base extends LinearOpMode {

    //Drive
    protected MecanumDrive drive;
    public static PIDCoefficients H_PID = new PIDCoefficients(-2, 0, -0.01);
    protected DcMotorEx fLeft, fRight, bLeft, bRight;
    protected Gyroscope gyro;
    private boolean initializedMotors = false, initializedDrive = false, initializedGyro = false;

    //Control objects
    protected BulkRead bReadCH, bReadEH;
    private boolean hasCH, hasEH;

    protected boolean USE_SUBS;

    public void driveTo(double destX, double destY, double destH) {
        PIDFController hControl = new PIDFController(H_PID);
        hControl.setOutputBounds(0, 1);
        hControl.setTargetPosition(0);

        //How accurate each attribute should be at each point
        double xAcc = 1, yAcc = 1, hAcc = Math.toRadians(1);

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
                (Math.abs(relX) > xAcc ||
                        Math.abs(relY) > yAcc ||
                        Math.abs(relH) > hAcc)) {

            updateBulkRead();
            gyro.update();
            drive.update();

            //Update robot position
            robotX = drive.getPoseEstimate().getX();
            robotY = drive.getPoseEstimate().getY();
            robotH = drive.getPoseEstimate().getHeading();

            //Calculate relatives
            relX = destX - robotX;
            relY = destY - robotY;
            relH = deltaHeading(robotH, destH);

            hWeight = hControl.update(Math.abs(relH));

            if (Math.abs(relX) < xAcc) relX = 0;
            if (Math.abs(relY) < yAcc) relY = 0;
            if (Math.abs(relH) < hAcc) hWeight = 0;

            double driveAngle = deltaHeading(robotH, Math.atan2(relY, relX));

            double xPower = Math.cos(driveAngle);
            double yPower = Math.sin(driveAngle);
            double hPower = Math.copySign(hWeight, relH);

            if (Math.sqrt((relX*relX) + (relY*relY)) < 5) {
                xPower *= 0.25;
                yPower *= 0.25;
            }
            else {
                if (Math.abs(xPower) < 0.25 && Math.abs(xPower) > 0)
                    xPower = Math.copySign(0.25, xPower);
                if (Math.abs(yPower) < 0.25 && Math.abs(yPower) > 0)
                    yPower = Math.copySign(0.25, yPower);
                if (Math.abs(hPower) < 0.25 && Math.abs(hPower) > 0)
                    hPower = Math.copySign(0.25, hPower);
            }

            drive.setWeightedDrivePower(new Pose2d(xPower, yPower, hPower));

            telemetry.addData("X Power: ", xPower);
            telemetry.addData("Y Power: ", yPower);
            telemetry.addData("H Power: ", hPower);

            updateSubsystems();
            updateTelemetry();
        }

        setMotorPowers(0, 0, 0, 0);
    }

    /**
     * Moves to a specific position without using Roadrunner trajectories
     * @param destX The destination x position
     * @param destY The destination y position
     */
    public void moveTo(double destX, double destY) {
        double scale = 0.25;

        double relativeX = destX - (-drive.getPoseEstimate().getY());
        double relativeY = destY - (drive.getPoseEstimate().getX());

        double startTime = System.currentTimeMillis();
        while (opModeIsActive() && (Math.abs(relativeX) > 0.1 || Math.abs(relativeY) > 0.1) && System.currentTimeMillis() < startTime + 1000) {

            //Update bulk read, odometry, and subsystems
            updateBulkRead();
            gyro.update();
            drive.update();

            //Output telemetry
            telemetry.addLine("Waiting for position correction");

            //Gets the distance to the point
            relativeX = destX - (-drive.getPoseEstimate().getY());
            relativeY = destY - (drive.getPoseEstimate().getX());

            //Angle the robot needs to drive in (in reference to its own front)
            double robotAngle = Math.atan2(relativeY, relativeX) - Math.toRadians(Math.toDegrees(drive.getPoseEstimate().getHeading()) + 90) + Math.PI / 4;

            //Calculates each motor power using trig
            double fL = scale * Math.cos(robotAngle);
            double fR = scale * Math.sin(robotAngle);
            double bL = scale * Math.sin(robotAngle);
            double bR = scale * Math.cos(robotAngle);

            //Sets powers to motors
            drive.setMotorPowers(fL, bL, bR, fR);

            updateSubsystems();
            updateTelemetry();
        }

        //Stops motors
        drive.setMotorPowers(0, 0, 0, 0);
    }

    /**
     * @param destAngle Destination angle
     * @param heading   Current angle
     * @return The shortest distance between two angles.
     */
    public double getDelta(double destAngle, double heading) {
        if (heading >= 360) {
            heading -= 360;
        }
        double delta;
        if (heading > destAngle) {
            delta = heading - destAngle;
            if (delta > 180) {
                return 360 - delta;
            }

            return -delta;
        }
        else {
            delta = destAngle - heading;
            if (delta > 180) {
                return -(360 - delta);
            }

            return delta;
        }
    }

    /**
     * Rotates slowly to a specific angle, without using Roadrunner trajectories
     * @param destAngle The destination angle
     */
    public void rotateTo(double destAngle) {

        double initAngle = Math.toDegrees(drive.getPoseEstimate().getHeading()); //Gets the first init angle

        //Variables for the rotate
        double speed, min=0.17, max=0.8;

        //Get current heading
        double heading = initAngle;
        double delta = getDelta(destAngle, heading);

        double startTime = System.currentTimeMillis();
        //If heading is not at destination
        while (opModeIsActive() && Math.abs(delta) > 0.125 && System.currentTimeMillis() < startTime + 1000) {

            //Update bulk read, odometry, and subsystems
            updateBulkRead();
            gyro.update();
            drive.update();

            //Output telemetry
            telemetry.addLine("Waiting for heading correction");

            //Gets heading
            heading = Math.toDegrees(drive.getPoseEstimate().getHeading());

            //Find delta
            delta = getDelta(destAngle, heading);

            //Finds the speed to rotate the robot so it doesn't over turn, Cubic function
            speed = 0.001 * Math.pow(delta,3);

            // Finds speeds boundaries
            if (speed > 0 && speed > max) {
                speed = max;
            }
            else if (speed < 0 && speed < -max) {
                speed = -max;
            }

            if (speed > 0 && speed < min) {
                speed = min;
            }
            else if (speed < 0 && speed > -min) {
                speed = -min;
            }

            //Drive the motors so the robot turns
            drive.setMotorPowers(-speed, -speed, speed, speed);

            updateSubsystems();
            updateTelemetry();
        }

        //Stops motors
        drive.setMotorPowers(0, 0, 0, 0);
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

    //Loop updates
    protected void updateBulkRead() {
        if (hasCH) bReadCH.update();
        if (hasEH) bReadEH.update();
    }
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
    protected void waitForDriveComplete() {
        while (opModeIsActive() && drive.isBusy()) {
            updateBulkRead();
            if (initializedGyro) gyro.update();
            if (initializedDrive) drive.update();
            updateSubsystems();
            updateTelemetry();
        }
    }

    protected void setMotorPowers(double v1, double v2, double v3, double v4) {
        fLeft.setPower(v1);
        fRight.setPower(v2);
        bLeft.setPower(v3);
        bRight.setPower(v4);
    }
}
