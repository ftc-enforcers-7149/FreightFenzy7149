package org.firstinspires.ftc.teamcode.Odometry.DriveWheels;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.util.Angle;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.Odometry.DriveWheels.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.Odometry.DriveWheels.DriveConstants.WHEEL_BASE;

public class MecanumLocalizer implements Localizer {

    private MecanumDrive drive;

    private Pose2d poseEstimate, poseVelocity;
    private List<Double> lastWheelPositions;
    private double lastExtHeading;

    public MecanumLocalizer(MecanumDrive drive) {
        this.drive = drive;

        poseEstimate = new Pose2d(0, 0, 0);
        poseVelocity = new Pose2d(0, 0, 0);
        lastWheelPositions = new ArrayList<Double>();

        lastWheelPositions.add(0d);
        lastWheelPositions.add(0d);
        lastWheelPositions.add(0d);
        lastWheelPositions.add(0d);

        lastExtHeading = 0;
    }

    @Override
    public void update() {
        //Only use in Tuning OpModes
        //drive.bReadCH.update();
        //drive.bReadEH.update();
        //drive.gyro.update();

        //Get current wheel positions & heading
        List<Double> wheelPositions = drive.getWheelPositions();
        double extHeading = drive.getRawExternalHeading();

        //Get change in wheel positions
        List<Double> wheelDeltas = new ArrayList<Double>();
        for (int i = 0; i < wheelPositions.size(); i++) {
            wheelDeltas.add(wheelPositions.get(i) - lastWheelPositions.get(i));
        }

        //Convert wheel position deltas to robot position delta
        Pose2d poseDelta = MecanumKinematics.wheelToRobotVelocities(
                wheelDeltas,
                TRACK_WIDTH,
                WHEEL_BASE,
                MecanumDrive.LATERAL_MULTIPLIER
        );

        //Get (and normalize) heading delta
        double extHeadingDelta = Angle.normDelta(extHeading - lastExtHeading);

        //Update robot position
        poseEstimate = Kinematics.relativeOdometryUpdate(
                poseEstimate,
                new Pose2d(poseDelta.vec(), extHeadingDelta));

        //Get current wheel velocities & angular velocity
        List<Double> wheelVelocities = drive.getWheelVelocities();
        double extHeadingVelocity = drive.getExternalHeadingVelocity();

        //Convert wheel velocities to robot velocity
        poseVelocity = MecanumKinematics.wheelToRobotVelocities(
                wheelVelocities,
                TRACK_WIDTH,
                WHEEL_BASE,
                MecanumDrive.LATERAL_MULTIPLIER
        );

        //Update robot velocity
        poseVelocity = new Pose2d(poseVelocity.vec(), extHeadingVelocity);

        //Update lasts for position deltas
        lastWheelPositions = wheelPositions;
        lastExtHeading = extHeading;
    }

    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        poseEstimate = pose2d;
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return poseVelocity;
    }

}
