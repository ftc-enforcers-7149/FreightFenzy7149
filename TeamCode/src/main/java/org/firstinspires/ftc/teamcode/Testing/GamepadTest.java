package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.TouchObjects.Slider;
import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.TouchObjects.Swipe;
import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.TouchObjects.TouchObject;
import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.TouchObjects.Button;
import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Touchpad;
import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Utils.Bounds;
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
        topRight = new Button(touchpad, false, Bounds.TOP_RIGHT, false);
        bottomLeft = new Button(touchpad, false, Bounds.BOTTOM_LEFT, true);
        liftPos = new Slider(touchpad, 0d, Slider.SliderType.Y_AXIS, 0, 20);
        rotateLeft = new Swipe(touchpad, false, Swipe.SwipeType.LEFT_SWIPE);
        rotateRight = new Swipe(touchpad, false, Swipe.SwipeType.RIGHT_SWIPE);

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

        if(touchpad.getNumFingers() >= 1)
            telemetry.addData("Finger 1: ", touchpad.getFingerOne());
        else
            telemetry.addLine();

        if(touchpad.getNumFingers() == 2)
            telemetry.addData("Finger 2: ", touchpad.getFingerTwo());
        else
            telemetry.addLine();

        telemetry.addLine();

        telemetry.addData("rotateLeft: ", rotateLeft.get());
        telemetry.addData("rotateRight: ", rotateRight.get());
        telemetry.addData("litPos: ", liftPos.get());

        telemetry.addLine();

        if(touchpad.getNumFingers() >= 1) telemetry.addData("V1: ", touchpad.getV1().getVelocity());
        telemetry.addData("V1 Angle: ", touchpad.getV1().getAngle(AngleUnit.DEGREES));
        if(touchpad.getNumFingers() == 2) telemetry.addData("V2: ", touchpad.getV2().getVelocity());
    }

    @Override
    protected void getInput() {

    }

    @Override
    protected void updateStateMachine() {

    }
}