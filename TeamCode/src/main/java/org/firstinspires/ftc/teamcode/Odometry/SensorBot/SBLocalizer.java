package org.firstinspires.ftc.teamcode.Odometry.SensorBot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.BulkRead;

import java.util.Arrays;
import java.util.List;

@Config
public class SBLocalizer extends ThreeTrackingWheelLocalizer {

    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = .74d; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double lMultiplier = 1.020070838d;
    public static double rMultiplier = 1.021469782d;
    public static double fMultiplier = 1.020456371d;

    public static double LATERAL_DISTANCE = 13.6992619926; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 0; // in; offset of the lateral wheel

    private DcMotorEx leftEncoder, rightEncoder, frontEncoder;
    private BulkRead bReadCH, bReadEH;

    public SBLocalizer(HardwareMap hardwareMap, BulkRead bReadCH, BulkRead bReadEH) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        leftEncoder = hardwareMap.get(DcMotorEx.class, "fRight");
        rightEncoder = hardwareMap.get(DcMotorEx.class,"bLeft");
        frontEncoder = hardwareMap.get(DcMotorEx.class,"fLeft"); //Correct

        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.bReadCH = bReadCH;
        this.bReadEH = bReadEH;
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                -encoderTicksToInches(bReadEH.getMotorPos(leftEncoder)) * lMultiplier,
                -encoderTicksToInches(bReadEH.getMotorPos(rightEncoder)) * rMultiplier,
                encoderTicksToInches(bReadEH.getMotorPos(frontEncoder)) * fMultiplier
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        return Arrays.asList(
                -encoderTicksToInches(bReadEH.getMotorVel(leftEncoder)),
                -encoderTicksToInches(bReadEH.getMotorVel(rightEncoder)),
                encoderTicksToInches(bReadEH.getMotorVel(frontEncoder))
        );
    }
}