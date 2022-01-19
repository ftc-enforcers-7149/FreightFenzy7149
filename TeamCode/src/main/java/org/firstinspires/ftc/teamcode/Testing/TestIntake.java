package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.TeleOp.TeleOp_Base;

public class TestIntake extends TeleOp_Base {
    //Headless
    private boolean resetAngle;

    //Intake
    Servo blocker, pusher;
    double blockerPosition, pusherPosition;

    //Motor
    DcMotor intake;
    double intakePower;

    public void init() {
        initializeDrive();
        driveHeadless(gyro.getYaw(), resetAngle);

        blocker = hardwareMap.servo.get("Blocker");
        pusher = hardwareMap.servo.get("Pusher");
        intake = hardwareMap.dcMotor.get("Intake");
    }

    public void loop() {
        //Headless
        leftX = curveInput(gamepad1.left_stick_x, 2)*lim;
        leftY = curveInput(gamepad1.left_stick_y, 2)*lim;
        rightX = curveInput(gamepad1.right_stick_x, 2)*lim*0.75;
        resetAngle = gamepad1.y;

        pusher.setPosition(pusherPosition);

        blocker.setPosition(blockerPosition);

        intake.setPower(intakePower);
    }

    public void stop() {
        stopInputs();
        stopOutputs();
    }

    @Override
    protected void getInput() {
        intakePower = gamepad1.right_trigger;

        if (gamepad1.a) {
            blockerPosition = 0.2;
        } else {
            blockerPosition = 0.8;
        }

        if (gamepad1.x) {
            pusherPosition = 0.2;
        } else {
            pusherPosition = 0.8;
        }
    }

    @Override
    protected void updateStateMachine() {

    }

}
