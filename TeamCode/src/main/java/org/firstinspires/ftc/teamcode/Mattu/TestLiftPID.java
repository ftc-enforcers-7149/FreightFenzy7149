package org.firstinspires.ftc.teamcode.Mattu;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.BulkRead;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.TeleOp.PrototypeChassis;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOp_Base;

@TeleOp(name = "Test Lift PID")
public class TestLiftPID extends TeleOp_Base {

    private Lift lift;
    private BulkRead bRead;

    private double liftPower;

    private enum LiftPosition {
        GROUND, LOW, MIDDLE, HIGH;
    }
    private LiftPosition liftPos, lastLiftPos;

    @Override
    public void init() {
        bRead = new BulkRead(hardwareMap, "Expansion Hub");
        lift = new Lift(hardwareMap, "lift", bRead);
    }

    @Override
    public void loop() {
        getInput();
        bRead.update();

        if (liftPower != 0) {
            lift.setPower(liftPower);
        }

        if (liftPos != lastLiftPos) {
            switch (liftPos) {
                case HIGH:
                    lift.setTargetHeight(15.75);
                    break;
                case MIDDLE:
                    lift.setTargetHeight(9.5);
                    break;
                case LOW:
                    lift.setTargetHeight(4);
                    break;
                case GROUND:
                    lift.setTargetHeight(0);
                    break;
            }
        }

        lift.update();

        telemetry.addData("Lift Height (in): ", lift.getLiftHeight());
        telemetry.addData("Lift Motor Ticks: ", lift.getMotorTicks());
        telemetry.addData("Target Position", liftPos);

        updateStateMachine();
    }

    @Override
    public void stop() {
        lift.stop();
    }

    @Override
    protected void getInput() {
        //Lift
        if (gamepad1.right_trigger > 0.1 || gamepad1.left_trigger > 0.1)
            liftPower = gamepad1.right_trigger - gamepad1.left_trigger;

        if (gamepad1.dpad_up) liftPos = LiftPosition.HIGH;
        else if (gamepad1.dpad_left) liftPos = LiftPosition.MIDDLE;
        else if (gamepad1.dpad_right) liftPos = LiftPosition.LOW;
        else if (gamepad1.dpad_down) liftPos = LiftPosition.GROUND;
    }

    @Override
    protected void updateStateMachine() {
        lastLeftX = leftX; lastLeftY = leftY; lastRightX = rightX;
        lastLiftPos = liftPos;
    }
}
