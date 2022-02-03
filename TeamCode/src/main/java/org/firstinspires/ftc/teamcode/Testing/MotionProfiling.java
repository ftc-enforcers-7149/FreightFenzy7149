package org.firstinspires.ftc.teamcode.Testing;

import android.os.SystemClock;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MotionProfiling extends OpMode {
    DcMotor spinner;
    PIDFController controller;
    double elapsedTime, currentTime = 0, lastTime = 0, lastX = 0;
    //*************************Get conversion from encoder ticks to rotations*******************************
    MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(
            new MotionState(0, 0, 0),
            new MotionState(60, 2, 2), //Set X to rotations
            2,
            2,
            2
    );

    @Override
    public void init() {
        spinner = hardwareMap.dcMotor.get("spinner");
        spinner.setDirection(DcMotorSimple.Direction.FORWARD);

        //Initialise PIDF
    }

    @Override
    public void loop() {
        currentTime = SystemClock.currentThreadTimeMillis();
        elapsedTime = currentTime - lastTime;

        MotionState state = profile.get(elapsedTime);

        double measuredPosition = state.getX() - lastX; //Make this the actual rotations of the motor

        controller.setTargetPosition(state.getX());
        controller.setTargetVelocity(state.getV());
        controller.setTargetAcceleration(state.getA());

        double correction = controller.update(measuredPosition);
        spinner.setPower(correction);

        lastTime = currentTime;
        lastX = state.getX();
    }

    @Override
    public void stop() {

    }

}
