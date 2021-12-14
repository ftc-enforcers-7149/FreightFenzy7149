package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.TouchObjects.Slider;
import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.TouchObjects.Swipe;
import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.TouchObjects.TouchObject;
import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.TouchObjects.Zone;
import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Touchpad;

import java.util.ArrayList;
import java.util.Arrays;

@TeleOp(name="Gamepad Test")
//@Disabled
public class GamepadTest extends OpMode {

    Touchpad touchpad;

    Zone topRight, bottomLeft;
    Slider liftPos;
    Swipe rotateLeft, rotateRight;

    public void init() {

        touchpad = new Touchpad(gamepad1);
        topRight = new Zone("topRight", touchpad, false, 0, 100, 0, 100);
        bottomLeft = new Zone("bottomLeft", touchpad, true, -100, 0, -100, 0);
        liftPos = new Slider("liftPos", touchpad, 1, TouchObject.Type.Y_AXIS);
        rotateLeft = new Swipe("rotateLeft", touchpad, 1, TouchObject.Type.LEFT_SWIPE);
        rotateRight = new Swipe("rotateRight", touchpad, 1, TouchObject.Type.RIGHT_SWIPE);

        try {
            touchpad.add(new ArrayList<>(Arrays.asList(topRight, bottomLeft, liftPos, rotateLeft, rotateRight)));
        }
        catch(Touchpad.DuplicateNameException ignored){}

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

        telemetry.addData("rotateLeft: ", rotateLeft.get());
        telemetry.addData("rotateRight: ", rotateRight.get());
        telemetry.addData("litPos: ", liftPos.get());

        if(touchpad.getNumFingers() >= 1) telemetry.addData("Swipe?: ", touchpad.getV1().getVelocity()); telemetry.addData("Angle?: ", touchpad.getV1().getAngle());
        if(touchpad.getNumFingers() == 2) telemetry.addData("Swipe 2?: ", touchpad.getV2().getVelocity());

        //touchpad.rumbleBlips(25);

    }

}