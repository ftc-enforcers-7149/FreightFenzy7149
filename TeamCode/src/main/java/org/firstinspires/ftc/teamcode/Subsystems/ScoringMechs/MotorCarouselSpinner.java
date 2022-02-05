package org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs;

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
import org.firstinspires.ftc.teamcode.Testing.MotionProfiling;

public class MotorCarouselSpinner implements Output, Input {
    DcMotor spinner;
    final double ticksPerRot = 384.5;
    double offset = 0;

    private PIDFController controller;
    public static PIDCoefficients coeffs = new PIDCoefficients(0.3, 0, 0);

    public enum MotorStates{
        MOTION_PROFILING(),
        FULL_POWER(),
        IDLE()
    }
    MotionProfiling.MotorStates mState = MotionProfiling.MotorStates.IDLE;

    MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(
            new MotionState(0, 0, 0),
            new MotionState((15*Math.PI - 1) / (4 * Math.PI) * 2, 10, 50), //Set X to rotations
            10,
            50
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
    public void updateInput() {

    }

    @Override
    public void updateOutput() {

    }
}
