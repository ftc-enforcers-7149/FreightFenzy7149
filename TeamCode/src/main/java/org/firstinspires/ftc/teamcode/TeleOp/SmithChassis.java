package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Smith Chassis")
//@Disabled
@Config
public class SmithChassis extends TeleOp_Base {

    public static double accelTime = 250;

    public void init() {
        initializeDrive();
        initializeVars();
    }

    public void loop() {
        getInput();
        driveAccelTank(accelTime, gamepad1.a);
        updateStateMachine();
    }

    public void stop() {
        setMotorPowers(0,0,0,0);
    }

    @Override
    protected void getInput() {
        leftY = gamepad1.left_stick_y;
        rightY = gamepad1.right_stick_y;
        time = System.currentTimeMillis();
    }

    @Override
    protected void updateStateMachine() {
        lastLeftY = leftY; lastRightY = rightY; lastTime = time;
    }
}
