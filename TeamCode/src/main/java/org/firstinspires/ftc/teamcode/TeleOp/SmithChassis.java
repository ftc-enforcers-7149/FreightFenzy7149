package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Smith Chassis")
//@Disabled
public class SmithChassis extends TeleOp_Base{

    public void init() {
        initializeDrive();
        initializeVars();
    }

    public void loop() {
        getInput();
        driveTank();
        updateStateMachine();
    }

    public void stop() {
        setMotorPowers(0,0,0,0);
    }

    @Override
    protected void getInput() {
        leftY = gamepad1.left_stick_y;
        rightY = gamepad1.right_stick_y;
    }

    @Override
    protected void updateStateMachine() {
        lastLeftY = leftY; lastRightX = rightX;
    }
}