package org.firstinspires.ftc.teamcode.Testing.CrashDetection;

import android.util.Log;

import com.acmerobotics.roadrunner.drive.TankDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Subsystems.Sensors.Gyroscope;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOp_Base;

@TeleOp(name="Crash Detection Logging")
public class CrashLog extends TeleOp_Base {

    protected DcMotorEx fLeft, fRight, bLeft, bRight;
    protected Gyroscope gyro;

    String[] steps = new String[]{"Drive straight and stop", "Strafe and stop", "Drive straight and coast", "Strafe and coast", "Turn",

                                  "Drive straight into wall", "Strafe into wall", "Hit from front", "Hit from sides", "Drive over barrier",
                                  "Hit barrier with right set of wheels", "Clipped by object"};
    int step = 0;

    boolean nextStep = false, x = false, lastX = false;
    boolean restartStep = false, a = false, lastA = false;

    @Override
    public void init() {

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

        gyro = new Gyroscope(hardwareMap);

    }

    @Override
    public void loop() {

        if(step == 0) Log.v("\n\n\nBeginning step", steps[step]);

        if(nextStep) {
            step++;
            Log.v("\n\n\nBeginning step", steps[step]);
        }

        if(restartStep) {
            Log.v("\n\n\nRestarting step", steps[step]);
        }

        getInput();

        telemetry.addData("Current step: ", steps[step]);
        telemetry.addLine("Press X to finish this step of testing and go to the next one.\n" +
                                    "Press A to repeat this step.");

        driveTank();
        updateStateMachine();

    }

    @Override
    public void getInput() {

        leftX = gamepad1.left_stick_x;
        leftY = gamepad1.left_stick_y;
        rightX = gamepad1.right_stick_x;
        rightY = gamepad1.right_stick_y;

        x = gamepad1.x;
        a = gamepad1.a;

        nextStep = x && x != lastX;
        restartStep = a && a != lastA;

    }

    @Override
    public void updateStateMachine() {

        lastLeftX = leftX;
        lastLeftY = leftY;
        lastRightX = rightX;
        lastRightY = rightY;

        lastX = x;
        lastA = a;

    }

}
