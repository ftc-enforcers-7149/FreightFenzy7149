package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Odometry.DriveWheels.MecanumDrive;
import org.firstinspires.ftc.teamcode.Odometry.SensorBot.SBMecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.BulkRead;
import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;

import static org.firstinspires.ftc.teamcode.Subsystems.FixedRoadrunner.createPose2d;

public abstract class TeleOp_Base extends OpMode {

    //Drive
    protected MecanumDrive drive;
    protected DcMotorEx fLeft, fRight, bLeft, bRight;
    private boolean initializedDrive = false;

    //Control objects
    protected BulkRead bReadCH, bReadEH;
    private boolean hasCH, hasEH;

    //State machine logic
    protected double leftX, leftY, rightX;
    protected double lastLeftX, lastLeftY, lastRightX;

    //Headless
    protected double offset, lim;

    //Initialization
    protected void initializeDrive() {
        fLeft = hardwareMap.get(DcMotorEx.class, "fLeft");
        fRight = hardwareMap.get(DcMotorEx.class, "fRight");
        bLeft = hardwareMap.get(DcMotorEx.class, "bLeft");
        bRight = hardwareMap.get(DcMotorEx.class, "bRight");

        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);

        fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        initializedDrive = true;
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
    protected void initializeOdometry() throws Exception {
        if (!hasCH) throw new Exception("Missing \"Control Hub\". Check configuration file naming");
        if (initializedDrive)
            drive = new MecanumDrive(hardwareMap, bReadCH, fLeft, fRight, bLeft, bRight);
        else
            drive = new MecanumDrive(hardwareMap, bReadCH);
        drive.setPoseEstimate(createPose2d(0, 0, 0));
    }
    protected void initializeVars() {
        lastLeftX = 0; lastLeftY = 0; lastRightX = 0;
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
        double v1 = -leftY + leftX - rightX;
        double v2 = -leftY - leftX + rightX;
        double v3 = -leftY - leftX - rightX;
        double v4 = -leftY + leftX + rightX;

        //Getting the max value can assure that no motor will be set to a value above a certain point.
        double max = Math.max(Math.max(Math.abs(v1), Math.abs(v2)), Math.max(Math.abs(v3), Math.abs(v4)));

        //In this case, no motor can go above lim power by scaling them all down if such a thing might occur.
        if (max > 1) {
            v1 /= max;
            v2 /= max;
            v3 /= max;
            v4 /= max;
        }

        setMotorPowers(v1, v2, v3, v4);
    }
    protected void driveHeadless(double angle, boolean reset) {
        if (reset) {
            offset = angle;
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
            vFL /= max * (1 / lim);
            vFR /= max * (1 / lim);
            vBL /= max * (1 / lim);
            vBR /= max * (1 / lim);
        }

        fLeft.setPower(vFL);
        fRight.setPower(vFR);
        bLeft.setPower(vBL);
        bRight.setPower(vBR);
    }

    //State machine logic
    protected abstract void getInput();
    protected abstract void updateStateMachine();

    //Useful functions
    protected void setMotorPowers(double v1, double v2, double v3, double v4) {
        fLeft.setPower(v1);
        fRight.setPower(v2);
        bLeft.setPower(v3);
        bRight.setPower(v4);
    }
}