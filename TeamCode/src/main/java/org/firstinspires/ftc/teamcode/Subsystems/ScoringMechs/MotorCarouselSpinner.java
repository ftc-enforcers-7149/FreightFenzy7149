package org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs;

import android.os.SystemClock;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors.BulkRead;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Input;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Output;

public class MotorCarouselSpinner implements Output, Input {

    public DcMotorEx spinner;
    private final double ticksPerRot = 384.5;
    private double offset = 0;

    private PIDFController controller;
    public static PIDCoefficients coeffs = new PIDCoefficients(0.3, 0, 0);

    private BulkRead bRead;
    private boolean useBR;

    private double elapsedTime, currentTime = 0, startTime = 0;
    private double measuredPosition;

    public enum MotorStates{
        MOTION_PROFILING,
        FULL_POWER,
        DUCK_PROFILING,
        IDLE
    }
    private MotorStates mState = MotorStates.IDLE;

    private MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(
            new MotionState(0, 0, 0),
            new MotionState((15*Math.PI - 1) / (4 * Math.PI) * 2, 9, 200),
            9,
            200
    );

    private MotionProfile profileSlow = MotionProfileGenerator.generateSimpleMotionProfile(
            new MotionState(0, 0, 0),
            new MotionState((15*Math.PI - 1) / (4 * Math.PI) * 2, 3.75, 150),
            3.75,
            150
    );

    private boolean slowProfile = false;

    private double power, lastPower;

    public MotorCarouselSpinner(HardwareMap hardwareMap, String spinnerName, BulkRead bRead, Alliance alliance) {
        spinner = hardwareMap.get(DcMotorEx.class, spinnerName);
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.bRead = bRead;
        useBR = true;

        if (alliance == Alliance.BLUE)
            spinner.setDirection(DcMotorSimple.Direction.FORWARD);
        else
            spinner.setDirection(DcMotorSimple.Direction.REVERSE);

        //Initialise PIDF
        controller = new PIDFController(coeffs);
        controller.setOutputBounds(-1, 1);
    }

    public MotorCarouselSpinner(HardwareMap hardwareMap, String spinnerName, Alliance alliance) {
        spinner = hardwareMap.get(DcMotorEx.class, spinnerName);
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        useBR = false;

        if (alliance == Alliance.BLUE)
            spinner.setDirection(DcMotorSimple.Direction.FORWARD);
        else
            spinner.setDirection(DcMotorSimple.Direction.REVERSE);

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

    public void resetSlow() {
        reset();
        slowProfile = true;
    }

    @Override
    public void updateInput() {
        currentTime = SystemClock.currentThreadTimeMillis();
        measuredPosition = spinner.getCurrentPosition() / ticksPerRot - offset;
    }

    @Override
    public void updateOutput() {
        if (!slowProfile) {
            if (measuredPosition < (15 * Math.PI - 1) / (4 * Math.PI)) {
                mState = MotorStates.MOTION_PROFILING;
            } else if (measuredPosition < (15 * Math.PI - 1) / (4 * Math.PI) + 1) {
                mState = MotorStates.FULL_POWER;
            } else {
                mState = MotorStates.IDLE;
            }
        }
        else {
            if (measuredPosition < (15 * Math.PI - 1) / (4 * Math.PI)) {
                mState = MotorStates.DUCK_PROFILING;
            } else if (measuredPosition < (15 * Math.PI - 1) / (4 * Math.PI) + 1) {
                mState = MotorStates.FULL_POWER;
            } else {
                mState = MotorStates.IDLE;
            }
        }

        MotionState state;
        double correction;


        switch (mState) {
            case MOTION_PROFILING:
                elapsedTime = currentTime - startTime;

                state = profile.get(elapsedTime / 1000.0);

                controller.setTargetPosition(state.getX());
                controller.setTargetVelocity(state.getV());
                controller.setTargetAcceleration(state.getA());

                correction = controller.update(measuredPosition);
                if (Math.abs(correction) > 0.01)
                    power = correction;
                else
                    power = 0;
                break;
            case DUCK_PROFILING:
                elapsedTime = currentTime - startTime;

                state = profileSlow.get(elapsedTime / 1000.0);

                controller.setTargetPosition(state.getX());
                controller.setTargetVelocity(state.getV());
                controller.setTargetAcceleration(state.getA());

                correction = controller.update(measuredPosition);
                if (Math.abs(correction) > 0.01)
                    power = correction;
                else
                    power = 0;
                break;
            case FULL_POWER:
                power = 1;
                slowProfile = false;
                break;
            case IDLE:
                power = 0;
                break;
        }

        if (power != lastPower) {
            spinner.setPower(power);
        }

        lastPower = power;
    }

    public boolean isBusy() {
        return mState != MotorStates.IDLE;
    }

    public double returnTargetPosition() {
        return controller.getTargetPosition();
    }

    public double returnCurrentPosition() {
        return spinner.getCurrentPosition() / ticksPerRot - offset;
    }

    public double returnCurrentVelocity() {
        return spinner.getVelocity() / ticksPerRot;
    }

    public double returnTargetVelocity() {
        return controller.getTargetVelocity();
    }

    @Override
    public void stopOutput() {
        startTime = Integer.MAX_VALUE;
        updateOutput();
    }
}
