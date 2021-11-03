package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Smith Chassis")
@Disabled
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

        telemetry.addData("Left Y:, ", gamepad1.left_stick_y);
        telemetry.addData("Left Power:", fLeft.getPower());
        telemetry.addData("Left state: ", aStateL);
        telemetry.addData("Left last state: ", lastAStateL);
        telemetry.addData("\nRight Y: ", gamepad1.right_stick_y);
        telemetry.addData("Right Power: ", fRight.getPower());
        telemetry.addData("Right state: ", aStateR);
        telemetry.addData("Right last state: ", lastAStateR);

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
