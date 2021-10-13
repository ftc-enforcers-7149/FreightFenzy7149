package org.firstinspires.ftc.teamcode.NovaLmao;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="GamepadTest")
public class GamepadTest extends OpMode {

    GamepadFun gamepadFun;

    public void init() {
        gamepadFun = new GamepadFun(gamepad1);
    }

    public void loop() {

        gamepadFun.update();
        telemetry.addData("Touch button?: ", (gamepadFun.isTouchButton()) ? "Yes" : "No");
        telemetry.addData("Number of fingers: ", gamepadFun.getNumFingers());
        if(gamepadFun.getNumFingers() >= 1) telemetry.addData("Finger 1 X?: ", gamepadFun.getFingerOneX());
        if(gamepadFun.getNumFingers() == 2) telemetry.addData("Finger 2 X?: ", gamepadFun.getFingerTwoX());
        if(gamepadFun.getNumFingers() >= 1) telemetry.addData("Finger 1 Y?: ", gamepadFun.getFingerOneY());
        if(gamepadFun.getNumFingers() == 2) telemetry.addData("Finger 2 Y?: ", gamepadFun.getFingerTwoY());

        //gamepadFun.rumble(25);
    }
}
