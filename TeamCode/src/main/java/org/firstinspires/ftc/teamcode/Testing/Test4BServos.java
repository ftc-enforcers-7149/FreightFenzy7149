package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test 4B Servos")
@Disabled
public class Test4BServos extends OpMode {

    Servo fourBarL, fourBarR, servo;

    boolean lastUp, lastDown;

    double position;

    @Override
    public void init() {
        fourBarL = hardwareMap.servo.get("fourBarL");
        fourBarL.setDirection(Servo.Direction.REVERSE);

        fourBarR = hardwareMap.servo.get("fourBarR");

        servo = fourBarL;

        position = 0;
    }

    @Override
    public void init_loop() {
        if (gamepad1.a)
            servo = fourBarL;
        if (gamepad1.b)
            servo = fourBarR;

        telemetry.addData("Servo: ", servo.getDeviceName());
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up && !lastUp)
            position += 0.01;
        else if (gamepad1.dpad_down && !lastDown)
            position -= 0.01;

        if (position < 0) position = 0;
        else if (position > 1) position = 1;

        servo.setPosition(position);

        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;

        telemetry.addData("Position: ", position);
    }
}
