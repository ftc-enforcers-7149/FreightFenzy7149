package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Slider;
import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.TouchZone;
import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Touchpad;
import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.TouchObject;

import java.util.ArrayList;
import java.util.HashMap;

@TeleOp(name="Gamepad Test")
//@Disabled
public class GamepadTest extends OpMode {

    Touchpad touchpad;

    boolean rotateLeft, rotateRight;
    double liftPos;

    public void init() {

        touchpad = new Touchpad(gamepad1);
        touchpad.add(new TouchZone("topRight", touchpad, 0, 100, 0, 100));
        touchpad.add(new TouchZone("bottomLeft", touchpad, -100, 0, -100, 0));
        touchpad.add(new Slider("liftPos", touchpad, 1, 0, 1, Slider.SwipeType.VERT_AXIS));

    }

    public void loop() {

        touchpad.update();

        telemetry.addData("Number of fingers: ", touchpad.getNumFingers());

        if(touchpad.getNumFingers() >= 1) {
            telemetry.addData("\nFinger 1 X?: ", touchpad.getFingerOneX());
            telemetry.addData("Finger 1 Y?: ", touchpad.getFingerOneY());
        }

        telemetry.addLine(touchpad.getTouchObjects().toString());

        if(touchpad.getNumFingers() == 2) {
            telemetry.addData("\nFinger 2 X?: ", touchpad.getFingerTwoX());
            telemetry.addData("Finger 2 Y?: ", touchpad.getFingerTwoY());
        }

        telemetry.addData("rotateLeft: ", rotateLeft);
        telemetry.addData("rotateRight: ", rotateRight);
        telemetry.addData("litPos: ", liftPos);

        if(touchpad.getNumFingers() >= 1) telemetry.addData("Swipe?: ", touchpad.getV1().getVelocity()); telemetry.addData("Angle?: ", touchpad.getV1().getAngle());
        if(touchpad.getNumFingers() == 2) telemetry.addData("Swipe 2?: ", touchpad.getV2().getVelocity());

        //touchpad.rumbleBlips(25);

    }

    public void getInput() {

        rotateLeft = (boolean) touchpad.getObject("rotateLeft").update();

    }

}