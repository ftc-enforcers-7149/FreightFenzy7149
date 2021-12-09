package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.TouchButton;
import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Touchpad;

@TeleOp(name="Gamepad Test")
//@Disabled
public class GamepadTest extends OpMode {

    Touchpad touchpad;

    public void init() {
        touchpad = new Touchpad(gamepad1);
    }

    public void loop() {

        touchpad.update();

        telemetry.addData("Touch button?: ", (touchpad.isTouchButton()) ? "Yes" : "No");
        telemetry.addData("Number of fingers: ", touchpad.getNumFingers());

        if(touchpad.getNumFingers() >= 1) {
            telemetry.addData("\nFinger 1 X?: ", touchpad.getFingerOneX());
            telemetry.addData("Finger 1 Y?: ", touchpad.getFingerOneY());
        }

        if(touchpad.getNumFingers() == 2) {
            telemetry.addData("\nFinger 2 X?: ", touchpad.getFingerTwoX());
            telemetry.addData("Finger 2 Y?: ", touchpad.getFingerTwoY());
        }

        telemetry.addData("\nSwipe?: ", touchpad.getVelocity());

        if(touchpad.getNumFingers() >= 1) telemetry.addData("Swipe?: ", touchpad.getV1().getVelocity());
        if(touchpad.getNumFingers() == 2) telemetry.addData("Swipe 2?: ", touchpad.getV2().getVelocity());

        touchpad.rumbleBlips(5);
    }
}
