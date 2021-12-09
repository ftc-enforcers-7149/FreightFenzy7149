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
    TouchButton leftButton, rightButton;

    public void init() {
        touchpad = new Touchpad(gamepad1);
        leftButton = new TouchButton(touchpad, -100, 0, -500, 500, 200);
        rightButton = new TouchButton(touchpad, 0, 100, -500, 500, 200);
    }

    public void loop() {

        touchpad.update();
        leftButton.update();
        rightButton.update();

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

        telemetry.addData("\nLeft button?: ", leftButton.isRange());
        telemetry.addData("Right button?", rightButton.isRange());

        telemetry.addData("\nLeft swipe?: ", leftButton.getV1().getVelocity() != 0);
        telemetry.addData("Right swipe?: ", rightButton.getV1().getVelocity() != 0);

        telemetry.addData("\nLeft vel?: ", leftButton.getV1().getVelocity());
        telemetry.addData("Right vel?: ", rightButton.getV1().getVelocity());

        touchpad.rumbleBlips(5);
    }
}
