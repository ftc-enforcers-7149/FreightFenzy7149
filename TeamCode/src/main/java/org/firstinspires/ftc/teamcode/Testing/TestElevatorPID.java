package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.ElevatorOld;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOp_Base;

@TeleOp(name = "Test Elevator PID")
@Disabled
public class TestElevatorPID extends TeleOp_Base {

    private ElevatorOld elevator;

    private double elevatorPower;

    private ElevatorOld.Level elevatorPos, lastElevatorPos;

    @Override
    public void init() {
        initializeBulkRead();

        elevator = new ElevatorOld(hardwareMap, "elevator", bReadEH);

        addInput(elevator);
        addOutput(elevator);
    }

    @Override
    public void loop() {
        updateInputs();
        getInput();

        if (elevatorPower != 0) {
            elevator.setPower(elevatorPower);
        }

        if (elevatorPos != lastElevatorPos) {
            elevator.setTargetHeight(elevatorPos);
        }

        telemetry.addData("Elevator Height (in): ", elevator.getHeight());
        telemetry.addData("Elevator Motor Ticks: ", elevator.getMotorTicks());
        telemetry.addData("Target Position", elevatorPos);

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
        //Elevator
        if (gamepad1.right_trigger > 0.1 || gamepad1.left_trigger > 0.1)
            elevatorPower = gamepad1.right_trigger - gamepad1.left_trigger;
        else
            elevatorPower = 0;

        if (gamepad1.dpad_up) elevatorPos = ElevatorOld.Level.HIGH;
        else if (gamepad1.dpad_left) elevatorPos = ElevatorOld.Level.MIDDLE;
        else if (gamepad1.dpad_right) elevatorPos = ElevatorOld.Level.LOW;
        else if (gamepad1.dpad_down) elevatorPos = ElevatorOld.Level.GROUND;
    }

    @Override
    protected void updateStateMachine() {
        lastLeftX = leftX; lastLeftY = leftY; lastRightX = rightX;
        lastElevatorPos = elevatorPos;
    }
}
