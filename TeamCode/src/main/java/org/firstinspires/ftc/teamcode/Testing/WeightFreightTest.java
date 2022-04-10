package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.MotorIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors.WeightFreightDetector;

@TeleOp(name="Weight Freight Test :00000")
@Disabled
public class WeightFreightTest extends OpMode {

    WeightFreightDetector w;
    double lastWeight = 0;

    MotorIntake i;

    public void init() {

        w = new WeightFreightDetector(hardwareMap, "force", 5);
        w.fsr406.setExcludeZero(true);
        i = new MotorIntake(hardwareMap, "intake", "paddle", "latch");

        i.startInput();
        i.startOutput();
    }

    public void loop() {

        w.updateInput();
        i.updateInput();

        i.setIntakePower(gamepad1.right_trigger);

        if (gamepad1.right_trigger == 0)
            i.setLatch(MotorIntake.LatchPosition.CLOSED);
        else
            i.setLatch(MotorIntake.LatchPosition.OPEN);

        telemetry.addData("Current type: ", w.getCurrentType());
        telemetry.addData("Current weight: ", w.getCurrentWeight());
        telemetry.addData("Delta W: ", (w.fsr406.getWeight() - lastWeight));
        telemetry.addData("Is freight in intake? ", w.isFreightInIntake());
        telemetry.addData("Get initial weight: ", w.getInitialWeight());

        lastWeight = w.fsr406.getWeight();

        i.updateOutput();
    }

    public void stop() {

        i.startInput();
        i.stopOutput();

    }

}
