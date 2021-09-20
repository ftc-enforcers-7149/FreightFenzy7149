package org.firstinspires.ftc.teamcode.Mattu;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;

@TeleOp(name = "Subsystems")
@Disabled
public class TestSubsystems extends OpMode {

    private Intake intake;

    @Override
    public void init() {
        intake = new Intake(hardwareMap, "intake", "ramp");
    }

    @Override
    public void loop() {
        intake.setPower(gamepad1.right_trigger);

        intake.update();
    }

    @Override
    public void stop() {
        intake.stop();
    }
}
