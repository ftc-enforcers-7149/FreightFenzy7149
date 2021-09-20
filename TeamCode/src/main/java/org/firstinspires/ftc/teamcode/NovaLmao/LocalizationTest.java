package org.firstinspires.ftc.teamcode.NovaLmao;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.DistanceLocalization;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOp_Base;

@TeleOp(name="Distance Sensor Localization")
//@Disabled
public class LocalizationTest extends TeleOp_Base {

    DistanceLocalization locale;

    public void init() {
        initializeDrive();
        initializeVars();

        locale = new DistanceLocalization(hardwareMap, "distF", "distR", 9);
    }

    public void loop() {
        getInput();
        driveArcade();
        locale.update();

        telemetry.addData("X: ", locale.getX());
        telemetry.addData("Y: ", locale.getY());
        telemetry.addData("Angle: ", locale.getTheta());
        telemetry.addData("Raw F: ", locale.getRawF());
        telemetry.addData("Raw R: ", locale.getRawR());
        telemetry.addData("Theoretical ratio: ", locale.getTheoreticalRatio());
        telemetry.addData("Actual ratio: ", locale.getActualRatio());
        telemetry.addData("Ratio difference: ", Math.abs(locale.getTheoreticalRatio() - locale.getActualRatio()));

        updateStateMachine();
    }

    public void stop() {
        setMotorPowers(0, 0, 0, 0);
    }

    public void getInput() {
        leftX = gamepad1.left_stick_x;
        leftY = gamepad1.left_stick_y;
        rightX = -Math.signum(gamepad1.right_stick_x) * Math.abs(Math.pow(gamepad1.right_stick_x, 3));
    }

    public void updateStateMachine() {
        lastLeftX = leftX; lastLeftY = leftY; lastRightX = rightX;
    }

}
