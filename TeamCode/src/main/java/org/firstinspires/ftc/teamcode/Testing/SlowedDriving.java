package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Elevator;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOp_Base;

@TeleOp(name = "Slowed Driving")
@Disabled
public class SlowedDriving extends TeleOp_Base {

    //Headless
    private boolean resetAngle;

    private Elevator elevator;

    @Override
    public void init() {
        initializeDrive();
        initializeBulkRead();
        initializeGyro();
        initializeVars();

        elevator = new Elevator(hardwareMap, "elevator", bReadEH);

        addInput(elevator);
        addOutput(elevator);
    }

    @Override
    public void loop() {
        updateInputs();
        getInput();

        // Drive
        driveHeadlessSmooth(gyro.getYaw(), resetAngle);

        // Elevator
        if (gamepad1.right_trigger > 0.1 || gamepad1.left_trigger > 0.1)
            elevator.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        else
            elevator.setPower(0);

        if (elevator.getHeight() > 10) {
            setSmoothingTimes(1, 200, 200);
        }
        else {
            setSmoothingTimes(300, 0, 0);
        }

        // Telemetry
        telemetry.addData("Elevator Height: ", elevator.getHeight());

        updateOutputs();
        updateStateMachine();
    }

    @Override
    public void stop() {
        stopInputs();
        stopOutputs();
        setMotorPowers(0, 0, 0, 0);
    }

    @Override
    protected void getInput() {
        //Headless
        leftX = gamepad1.left_stick_x;
        leftY = gamepad1.left_stick_y;
        rightX = gamepad1.right_stick_x;
        resetAngle = gamepad1.y;
    }

    @Override
    protected void updateStateMachine() {
        lastLeftX = leftX; lastLeftY = leftY; lastRightX = rightX;
    }
}
