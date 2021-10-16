package org.firstinspires.ftc.teamcode.NovaLmao;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="GamepadTest")
public class GamepadTest extends OpMode {

    GamepadFun gamepadFun;
    TouchButton leftButton, rightButton;

    public void init() {
        gamepadFun = new GamepadFun(gamepad1);
        leftButton = new TouchButton(gamepadFun, -100, 0, -500, 500);
        rightButton = new TouchButton(gamepadFun, 0, 100, -500, 500);
    }

    public void loop() {

        gamepadFun.update();
        leftButton.update();
        rightButton.update();

        telemetry.addData("Touch button?: ", (gamepadFun.isTouchButton()) ? "Yes" : "No");
        telemetry.addData("Number of fingers: ", gamepadFun.getNumFingers());

        if(gamepadFun.getNumFingers() >= 1) {
            telemetry.addData("\nFinger 1 X?: ", gamepadFun.getFingerOneX());
            telemetry.addData("Finger 1 Y?: ", gamepadFun.getFingerOneY());
        }

        if(gamepadFun.getNumFingers() == 2) {
            telemetry.addData("\nFinger 2 X?: ", gamepadFun.getFingerTwoX());
            telemetry.addData("Finger 2 Y?: ", gamepadFun.getFingerTwoY());
        }

        telemetry.addData("\nLeft button?: ", leftButton.isRange());
        telemetry.addData("Right button?", rightButton.isRange());

        telemetry.addData("\nLeft swipe?: ", leftButton.returnSwipeVector().getVelocity() != 0);
        telemetry.addData("Right swipe?: ", rightButton.returnSwipeVector().getVelocity() != 0);

        telemetry.addData("\nLeft swipe: ", leftButton.returnSwipeVector().getVelocity());
        telemetry.addData("Right swipe: ", rightButton.returnSwipeVector().getVelocity());

        gamepadFun.rumbleBlips(5);
    }
}
