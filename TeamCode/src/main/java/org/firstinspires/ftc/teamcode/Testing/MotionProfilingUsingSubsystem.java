package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.MotorCarouselSpinner;

public class MotionProfilingUsingSubsystem extends OpMode {
    MotorCarouselSpinner carouselSpinner;

    boolean a, lastA;

    @Override
    public void init() {
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
    }

    @Override
    public void stop() {
        carouselSpinner.stopOutput();
    }
}
