package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Sensor Bot")
@Disabled
public class SensorBotTele extends TeleOp_Base {

    @Override
    public void init() {
        initializeDrive();
        initializeBulkRead();
        initializeVars();
    }

    @Override
    public void loop() {
        updateBulkRead();
        getInput();

        driveArcade();

        updateStateMachine();
    }

    @Override
    public void stop() {
        setMotorPowers(0, 0, 0, 0);
    }

    @Override
    protected void getInput() {
        leftX = gamepad1.left_stick_x;
        leftY = gamepad1.left_stick_y;
        rightX = -Math.signum(gamepad1.right_stick_x) * Math.abs(Math.pow(gamepad1.right_stick_x, 3));
    }

    @Override
    protected void updateStateMachine() {
        lastLeftX = leftX; lastLeftY = leftY; lastRightX = rightX;
    }
}
