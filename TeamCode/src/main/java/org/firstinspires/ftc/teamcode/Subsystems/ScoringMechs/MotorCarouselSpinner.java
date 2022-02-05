package org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs;

import android.os.SystemClock;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Utils.Input;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Output;

public class MotorCarouselSpinner implements Output, Input {
    DcMotor spinner;
    final double ticksPerRot = 384.5;
    double offset = 0;

    private PIDFController controller;
    public static PIDCoefficients coeffs = new PIDCoefficients(0.3, 0, 0);

    double elapsedTime, currentTime = 0, startTime = 0;
    private double measuredPosition;

    public enum MotorStates{
        MOTION_PROFILING(),
        FULL_POWER(),
        IDLE()
    }
    MotorStates mState = MotorStates.IDLE;

    MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(
            new MotionState(0, 0, 0),
            new MotionState((15*Math.PI - 1) / (4 * Math.PI) * 2, 9, 200), //Set X to rotations
            9,
            200
    );

    public MotorCarouselSpinner (HardwareMap hardwareMap, String spinnerName) {
        spinner = hardwareMap.dcMotor.get(spinnerName);
        spinner.setDirection(DcMotorSimple.Direction.FORWARD);
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Initialise PIDF
        controller = new PIDFController(coeffs);
        controller.setOutputBounds(-1, 1);
    }

    @Override
    public void startOutput() {
        startTime = Integer.MAX_VALUE;
    }

    public void reset() {
        startTime = currentTime;
        offset = spinner.getCurrentPosition() / ticksPerRot;
        measuredPosition = 0;
    }

    @Override
    public void updateInput() {
        currentTime = SystemClock.currentThreadTimeMillis();
        measuredPosition = spinner.getCurrentPosition() / ticksPerRot - offset;
    }

    @Override
    public void updateOutput() {
        if (measuredPosition < (15*Math.PI - 1) / (4 * Math.PI)) {
            mState = MotorStates.MOTION_PROFILING;
        } else if (measuredPosition < (15*Math.PI - 1) / (4 * Math.PI) + 1) {
            mState = MotorStates.FULL_POWER;
        } else {
            mState = MotorStates.IDLE;
        }

        switch (mState) {
            case MOTION_PROFILING:
                elapsedTime = currentTime - startTime;

                MotionState state = profile.get(elapsedTime / 1000.0);

                controller.setTargetPosition(state.getX());
                controller.setTargetVelocity(state.getV());
                controller.setTargetAcceleration(state.getA());

                double correction = controller.update(measuredPosition);
                if (Math.abs(correction) > 0.01)
                    spinner.setPower(correction);
                else
                    spinner.setPower(0);
                break;
            case FULL_POWER:
                spinner.setPower(1);
                break;
            case IDLE:
                spinner.setPower(0);
                break;
        }
    }
}
