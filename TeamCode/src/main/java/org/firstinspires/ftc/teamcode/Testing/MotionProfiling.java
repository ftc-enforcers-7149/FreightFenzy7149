package org.firstinspires.ftc.teamcode.Testing;

import android.os.SystemClock;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
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
    public static PIDCoefficients pidCoeffs = new PIDCoefficients(0.01, 0, 0);
    private double elapsedTime, currentTime = 0, lastTime = 0, lastX = 0;
    private double wheelDiam = 4 * Math.PI;

    private double ticksPerRot = ((((1+(46/17))) * (1+(46/17))) * 28);

    MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(
            new MotionState(0, 0, 0),
            new MotionState(3.68, 2, 2),
            2,
            2,
            2
    );

    @Override
    public void init() {
        spinner = hardwareMap.dcMotor.get("spinner");
        spinner.setDirection(DcMotorSimple.Direction.FORWARD);

        //Initialise PIDF
        initPID();
    }

    @Override
    public void loop() {
        currentTime = SystemClock.currentThreadTimeMillis();
        elapsedTime = currentTime - lastTime;

        MotionState state = profile.get(elapsedTime);

        double measuredPosition = inchesToRotation(state.getX()) - lastX; //Make this the actual rotations of the motor

        controller.setTargetPosition(state.getX());
        controller.setTargetVelocity(state.getV());
        controller.setTargetAcceleration(state.getA());

        double correction = controller.update(measuredPosition);
        spinner.setPower(correction);

        lastTime = currentTime;
        lastX = inchesToRotation(state.getX());
    }

    @Override
    public void stop() {

    }

    private void initPID() {
        controller = new PIDFController(pidCoeffs);
        controller.setOutputBounds(-1, 1);
    }

    private double inchesToRotation (double inches) {
        return inches / wheelDiam;
    }
}
