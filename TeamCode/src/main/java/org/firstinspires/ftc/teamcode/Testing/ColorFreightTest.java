package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.MotorIntake;

@TeleOp(name="Color Freight Test")
public class ColorFreightTest extends OpMode {

    MotorIntake i;

    public void init() {

        i = new MotorIntake(hardwareMap, "intake", "paddle", "latch", "intakeColor");
        i.startInput();
        i.startOutput();

    }


    public void loop() {

        i.updateInput();
        i.setIntakePower(gamepad1.right_trigger);
        if(gamepad1.left_trigger > 0.10 || gamepad1.right_trigger > 0.1) i.setLatch(MotorIntake.LatchPosition.OPEN);
        else i.setLatch(MotorIntake.LatchPosition.CLOSED);

        telemetry.addData("Hue: ", i.sensor.getHue());
        telemetry.addData("Saturation: ", i.sensor.getSaturation());
        telemetry.addData("Value: ", i.sensor.getValue());
        telemetry.addData("Distance: ", i.getDistance());
        telemetry.addData("Type: ", i.getFreightType());

        i.updateOutput();

    }

}
