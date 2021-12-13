package org.firstinspires.ftc.teamcode.Testing;

import android.text.method.Touch;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.TouchButton;
import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Touchpad;

import java.util.HashMap;

@TeleOp(name="Gamepad Test")
//@Disabled
public class GamepadTest extends OpMode {

    Touchpad touchpad;
    HashMap<TouchButton, Boolean> touchButtons;

    public void init() {

        touchpad = new Touchpad(gamepad1);
        touchpad.addButton(new TouchButton("test" ,0, 100, 0, 100));

    }

    public void loop() {

        touchpad.update();
        touchButtons = touchpad.getTouchButtons();

        telemetry.addData("Touch button?: ", (touchpad.isTouchButton()) ? "Yes" : "No");
        telemetry.addData("Number of fingers: ", touchpad.getNumFingers());

        if(touchpad.getNumFingers() >= 1) {
            telemetry.addData("\nFinger 1 X?: ", touchpad.getFingerOneX());
            telemetry.addData("Finger 1 Y?: ", touchpad.getFingerOneY());

            telemetry.addData("Same X?: ", touchpad.getFingerOneX() == touchpad.getLastFingerOneX());
            telemetry.addData("Same Y: ", touchpad.getFingerOneY() == touchpad.getLastFingerOneY());
        }

        for (HashMap.Entry mapElement : touchButtons.entrySet()) {

            TouchButton key = (TouchButton) mapElement.getKey();
            telemetry.addData(key.getName() + ": ", mapElement.getValue());

        }

        if(touchpad.getNumFingers() == 2) {
            telemetry.addData("\nFinger 2 X?: ", touchpad.getFingerTwoX());
            telemetry.addData("Finger 2 Y?: ", touchpad.getFingerTwoY());
            telemetry.addData("\nL Finger 2 X?: ", touchpad.getLastFingerTwoX());
            telemetry.addData("L Finger 2 Y?: ", touchpad.getLastFingerTwoY());
        }

        if(touchpad.getNumFingers() >= 1) telemetry.addData("Swipe?: ", touchpad.getV1().getVelocity()); telemetry.addData("Angle?: ", touchpad.getV1().getAngle());
        if(touchpad.getNumFingers() == 2) telemetry.addData("Swipe 2?: ", touchpad.getV2().getVelocity());

        //touchpad.rumbleBlips(25);
    }
}
