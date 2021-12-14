package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.TouchObjects.Slider;
import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.TouchObjects.Swipe;
import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.TouchObjects.TouchObject;
import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.TouchObjects.Button;
import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Touchpad;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOp_Base;

@TeleOp(name="Gamepad Test")
//@Disabled
public class GamepadTest extends TeleOp_Base {

    Touchpad touchpad;

    Button topRight, bottomLeft;
    Slider liftPos;
    Swipe rotateLeft, rotateRight;

    public void init() {
        initializeSources();
        initializeVars();

        touchpad = new Touchpad(gamepad1);
        topRight = new Button(touchpad, false, false, 0, 100, 0, 100);
        bottomLeft = new Button(touchpad, false, true, -100, 0, -100, 0);
        liftPos = new Slider(touchpad, 0d, 1, TouchObject.Type.Y_AXIS);
        rotateLeft = new Swipe(touchpad, false, 1, TouchObject.Type.LEFT_SWIPE);
        rotateRight = new Swipe(touchpad, false, 1, TouchObject.Type.RIGHT_SWIPE);

        addInput(touchpad);
        addInput(topRight);
        addInput(bottomLeft);
        addInput(liftPos);
        addInput(rotateLeft);
        addInput(rotateRight);
    }

    public void loop() {
        updateInputs();

        telemetry.addData("Number of fingers: ", touchpad.getNumFingers());

        if(touchpad.getNumFingers() >= 1) {
            telemetry.addData("\nFinger 1 X?: ", touchpad.getFingerOneX());
            telemetry.addData("Finger 1 Y?: ", touchpad.getFingerOneY());
        }

        if(touchpad.getNumFingers() == 2) {
            telemetry.addData("\nFinger 2 X?: ", touchpad.getFingerTwoX());
            telemetry.addData("Finger 2 Y?: ", touchpad.getFingerTwoY());
        }

        telemetry.addData("rotateLeft: ", rotateLeft.get());
        telemetry.addData("rotateRight: ", rotateRight.get());
        telemetry.addData("litPos: ", liftPos.get());

        if(touchpad.getNumFingers() >= 1) telemetry.addData("Swipe?: ", touchpad.getV1().getVelocity());
        telemetry.addData("Angle?: ", touchpad.getV1().getAngle());
        if(touchpad.getNumFingers() == 2) telemetry.addData("Swipe 2?: ", touchpad.getV2().getVelocity());
    }

    @Override
    protected void getInput() {

    }

    @Override
    protected void updateStateMachine() {

    }
}