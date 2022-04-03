package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.hardware.stmicroelectronics.VL53L0X;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.ArmController;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.FourBar;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Lift;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOp_Base;

import static org.firstinspires.ftc.teamcode.GlobalData.RAN_AUTO;

@TeleOp(name = "Test Arm Controller")
//@Disabled
public class TestArmController extends TeleOp_Base {

    Lift lift;
    FourBar fourBar;
    ArmController controller;

    @Override
    public void init() {
        try {
            initializeAll();
        } catch (Exception e) {
            e.printStackTrace();
            requestOpModeStop();
            return;
        }

        lift = new Lift(hardwareMap, "lift", bReadCH, !RAN_AUTO);
        fourBar = new FourBar(hardwareMap, "fourBarL", "fourBarR",
                "counterL", "counterR");

        controller = new ArmController(lift, fourBar);

        addInput(lift);
        addOutput(lift);
        addOutput(fourBar);
    }

    @Override
    public void loop() {
        updateInputs();
        getInput();

        if (gamepad1.dpad_down) controller.setScorePos(ArmController.ScoringPosition.IN);
        else if (gamepad1.dpad_up) controller.setScorePos(ArmController.ScoringPosition.UP);
        else if (gamepad1.dpad_left) controller.setScorePos(ArmController.ScoringPosition.MIDDLE);
        else if (gamepad1.dpad_right) controller.setScorePos(ArmController.ScoringPosition.HIGH);

        updateStateMachine();
        updateOutputs();
    }

    @Override
    protected void getInput() {

    }

    @Override
    protected void updateStateMachine() {

    }
}
