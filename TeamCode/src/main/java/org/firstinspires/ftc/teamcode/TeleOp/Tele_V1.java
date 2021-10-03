package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;
import org.firstinspires.ftc.teamcode.Subsystems.TurningIntake;

@TeleOp (name = "Tele_V1")
public class Tele_V1 extends TeleOp_Base {

    //Headless
    protected Gyroscope gyro;
    private boolean resetAngle;

    private TurningIntake turningIntake;

    @Override
    public void init() {
        initializeDrive();
        initializeVars();

        gyro = new Gyroscope(hardwareMap);

        turningIntake = new TurningIntake(hardwareMap, "intake", "wrist");
    }

    @Override
    public void loop() {
        getInput();

        driveHeadless(gyro.getRawYaw(), resetAngle);

        if (gamepad2.dpad_up) turningIntake.setWristPosCenter();
        if (gamepad2.dpad_left) turningIntake.setWristPosLeft();
        if (gamepad2.dpad_right) turningIntake.setWristPosRight();

        if (gamepad2.right_trigger > 0.1) {
            turningIntake.setIntakePower(gamepad1.right_trigger);
        }
        else if (gamepad2.left_trigger > 0.1) {
            turningIntake.setIntakePower(gamepad1.left_trigger);
        } else {
            turningIntake.setIntakePower(0);
        }

        turningIntake.update();
        updateStateMachine();
    }

    @Override
    public void stop() {
        setMotorPowers(0,0,0,0);
    }

    @Override
    protected void getInput() {
        //Headless
        leftX = gamepad1.left_stick_x;
        leftY = gamepad1.left_stick_y;
        rightX = Math.signum(gamepad1.right_stick_x) * Math.abs(Math.pow(gamepad1.right_stick_x, 3));
        resetAngle = gamepad1.y;
    }

    @Override
    protected void updateStateMachine() {
        lastLeftX = leftX; lastLeftY = leftY; lastRightX = rightX;
    }
}
