package org.firstinspires.ftc.teamcode.Testing;

import android.os.SystemClock;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name = "Test Spinner Profile")
public class MotionProfiling extends OpMode {

    DcMotor spinner;
    final double ticksPerRot = 384.5;
    double offset = 0;

    public static PIDCoefficients coeffs = new PIDCoefficients(0.3, 0, 0);
    PIDFController controller;

    double elapsedTime, currentTime = 0, startTime = 0;

    //*************************Get conversion from encoder ticks to rotations*******************************
    MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(
            new MotionState(0, 0, 0),
            new MotionState((15*Math.PI - 1) / (4 * Math.PI) * 2, 10, 50), //Set X to rotations
            10,
            50
    );

    boolean a, lastA;

    @Override
    public void init() {
        spinner = hardwareMap.dcMotor.get("spinner");
        spinner.setDirection(DcMotorSimple.Direction.FORWARD);
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Initialise PIDF
        controller = new PIDFController(coeffs);
        controller.setOutputBounds(-1, 1);
    }

    @Override
    public void start() {
        startTime = Integer.MAX_VALUE;
    }

    @Override
    public void loop() {
        currentTime = SystemClock.currentThreadTimeMillis();

        a = gamepad1.a;
        if (a && !lastA) {
            startTime = SystemClock.currentThreadTimeMillis();
            offset = spinner.getCurrentPosition() / ticksPerRot;
        }

        elapsedTime = currentTime - startTime;

        MotionState state = profile.get(elapsedTime / 1000.0);

        double measuredPosition = spinner.getCurrentPosition() / ticksPerRot;

        controller.setTargetPosition(state.getX());
        controller.setTargetVelocity(state.getV());
        controller.setTargetAcceleration(state.getA());

        double correction = controller.update(measuredPosition - offset);
        if (Math.abs(correction) > 0.01)
            spinner.setPower(correction);
        else
            spinner.setPower(0);

        lastA = gamepad1.a;
    }

    @Override
    public void stop() {
        spinner.setPower(0);
    }
}
