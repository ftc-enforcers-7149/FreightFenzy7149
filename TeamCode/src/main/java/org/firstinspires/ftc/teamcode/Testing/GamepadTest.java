package org.firstinspires.ftc.teamcode.Testing;

import com.enforcers7149.touchpadplusplus.src.TouchObjects.Button;
import com.enforcers7149.touchpadplusplus.src.TouchObjects.Slider;
import com.enforcers7149.touchpadplusplus.src.TouchObjects.Snapback;
import com.enforcers7149.touchpadplusplus.src.TouchObjects.Swipe;
import com.enforcers7149.touchpadplusplus.src.Touchpad;
import com.enforcers7149.touchpadplusplus.src.Utils.Bounds.PolygonBounds;
import com.enforcers7149.touchpadplusplus.src.Utils.Bounds.RectBounds;
import com.enforcers7149.touchpadplusplus.src.Utils.Point;
import com.enforcers7149.touchpadplusplus.src.Utils.TouchpadHandler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOp_Base;

@TeleOp(name="Gamepad Test")
//@Disabled
public class GamepadTest extends TeleOp_Base {

    Touchpad touchpad;

    Button topRight, bottomLeft;
    Button polyButton;
    Slider liftPos;
    Snapback slidePos;
    Swipe rotateLeft, rotateRight;
    TouchpadHandler tHandle;

    public void init() {
        initializeSources();
        initializeVars();

        PolygonBounds polygon = new PolygonBounds(new Point(-100, 100), new Point(-100, 0),
                new Point(0, 50), new Point(100, 0), new Point(100, -100));

        touchpad = new Touchpad(gamepad1);
        topRight = new Button(touchpad, false, RectBounds.TOP_RIGHT, false);
        bottomLeft = new Button(touchpad, false, RectBounds.BOTTOM_LEFT, true);
        polyButton = new Button(touchpad, false, polygon, true);
        liftPos = new Slider(touchpad, 0, Slider.SliderType.Y_AXIS, 0, 20);
        rotateLeft = new Swipe(touchpad, false, Swipe.SwipeType.LEFT_SWIPE);
        rotateRight = new Swipe(touchpad, false, Swipe.SwipeType.RIGHT_SWIPE);
        slidePos = new Snapback(touchpad, 0, Slider.SliderType.X_AXIS, 0, 20);

        tHandle = new TouchpadHandler(touchpad, topRight, bottomLeft, polyButton, liftPos, rotateLeft, rotateRight, slidePos);

    }

    public void loop() {

        getInput();

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
        telemetry.addData("polyButton: ", polyButton.get());
        telemetry.addData("topRight: ", topRight.get());
        telemetry.addData("slidePos: ", slidePos.get());

        telemetry.addLine();

        if(touchpad.getNumFingers() >= 1) telemetry.addData("V1: ", touchpad.getV1().getVelocity());
        telemetry.addData("V1 Angle: ", touchpad.getV1().getAngle(AngleUnit.DEGREES));
        if(touchpad.getNumFingers() == 2) telemetry.addData("V2: ", touchpad.getV2().getVelocity());
    }

    @Override
    protected void getInput() {

        updateInputs();
        tHandle.updateInputs();

    }

    @Override
    protected void updateStateMachine() {

    }
}