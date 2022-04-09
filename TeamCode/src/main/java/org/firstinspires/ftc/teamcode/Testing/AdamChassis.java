package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleOp.TeleOp_Base;

@TeleOp(name="Adam Chassis")
public class AdamChassis extends TeleOp_Base {

    @Override
    public void init() {

        initializeSources();
        initializeDrive();
        initializeGyro();
        initializeVars();

    }

    @Override
    public void loop() {

        getInput();
        driveTank();
        updateStateMachine();

    }

    @Override
    protected void getInput() {

        leftY = gamepad1.left_stick_y;
        leftX = gamepad1.left_stick_x;
        rightY = gamepad1.right_stick_y;
        rightX = gamepad1.right_stick_x;

    }

    @Override
    protected void updateStateMachine() {

        lastLeftY = leftY;
        lastLeftX = leftX;
        lastRightY = rightY;
        lastRightX = rightX;

    }

}
