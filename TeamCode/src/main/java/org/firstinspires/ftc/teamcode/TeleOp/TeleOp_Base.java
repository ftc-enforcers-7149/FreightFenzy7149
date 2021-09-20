package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Odometry.SensorBot.SBMecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.BulkRead;

import static org.firstinspires.ftc.teamcode.Subsystems.FixedRoadrunner.createPose2d;

public abstract class TeleOp_Base extends OpMode {

    //Drive
    protected SBMecanumDrive drive;
    protected DcMotor fLeft, fRight, bLeft, bRight;

    //Control objects
    protected BulkRead bReadCH, bReadEH;
    private boolean hasCH, hasEH;

    //State machine logic
    protected double leftX, leftY, rightX;
    protected double lastLeftX, lastLeftY, lastRightX;

    //Initialization
    protected void initializeDrive() {
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");

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
        if (!hasCH || !hasEH) throw new Exception("Missing Hub");
        drive = new SBMecanumDrive(hardwareMap, bReadCH, bReadEH);
        drive.setPoseEstimate(createPose2d(0, 0, 0));
    }
    protected void initializeVars() {
        lastLeftX = 0; lastLeftY = 0; lastRightX = 0;
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
    protected void driveHeadless(double angle) {

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