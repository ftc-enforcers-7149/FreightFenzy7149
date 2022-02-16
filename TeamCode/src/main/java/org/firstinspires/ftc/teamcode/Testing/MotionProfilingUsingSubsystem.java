package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.MotorCarouselSpinner;

public class MotionProfilingUsingSubsystem extends OpMode {
    MotorCarouselSpinner carouselSpinner;

    boolean a, lastA;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        carouselSpinner = new MotorCarouselSpinner(hardwareMap, "spinner", Alliance.RED);
    }

    @Override
    public void start() {
        carouselSpinner.startOutput();
    }

    @Override
    public void loop() {
        a = gamepad1.a;
        carouselSpinner.updateInput();

        if (a && !lastA) {
            carouselSpinner.reset();
        }

        carouselSpinner.updateOutput();

        lastA = gamepad1.a;

        telemetry.addData("Current Position: ", carouselSpinner.returnCurrentPosition());
        telemetry.addData("Current Velocity: ", carouselSpinner.returnCurrentVelocity());
        telemetry.addData("Target Position: ", carouselSpinner.returnTargetPosition());
        telemetry.addData("Target Velocity: ", carouselSpinner.returnTargetVelocity());
    }

    @Override
    public void stop() {
        carouselSpinner.stopOutput();
    }
}
