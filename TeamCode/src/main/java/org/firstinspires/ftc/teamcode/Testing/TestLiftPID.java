package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Sensors.BulkRead;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Levels;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOp_Base;

@TeleOp(name = "Test Lift PID")
@Disabled
public class TestLiftPID extends TeleOp_Base {

    private Lift lift;
    private BulkRead bRead;

    private double liftPower;

    private Levels liftPos, lastLiftPos;

    @Override
    public void init() {
        initializeBulkRead();

        lift = new Lift(hardwareMap, "lift", bRead);

        addInput(lift);
        addOutput(lift);
    }

    @Override
    public void loop() {
        updateInputs();
        getInput();

        if (liftPower != 0) {
            lift.setPower(liftPower);
        }

        if (liftPos != lastLiftPos) {
            lift.setTargetHeight(liftPos);
        }

        telemetry.addData("Lift Height (in): ", lift.getHeight());
        telemetry.addData("Lift Motor Ticks: ", lift.getMotorTicks());
        telemetry.addData("Target Position", liftPos);

        updateOutputs();
        updateStateMachine();
    }

    @Override
    public void stop() {
        stopInputs();
        stopOutputs();
    }

    @Override
    protected void getInput() {
        //Lift
        if (gamepad1.right_trigger > 0.1 || gamepad1.left_trigger > 0.1)
            liftPower = gamepad1.right_trigger - gamepad1.left_trigger;
        else
            liftPower = 0;

        if (gamepad1.dpad_up) liftPos = Levels.HIGH;
        else if (gamepad1.dpad_left) liftPos = Levels.MIDDLE;
        else if (gamepad1.dpad_right) liftPos = Levels.LOW;
        else if (gamepad1.dpad_down) liftPos = Levels.GROUND;
    }

    @Override
    protected void updateStateMachine() {
        lastLeftX = leftX; lastLeftY = leftY; lastRightX = rightX;
        lastLiftPos = liftPos;
    }
}
