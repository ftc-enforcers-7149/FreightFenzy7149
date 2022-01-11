package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.Auto_V2;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Lift;

import java.util.function.Supplier;

@Autonomous(name = "Test Red Cycles")
//@Disabled
public class RedCycles extends Auto_V2 {

    @Override
    protected Alliance getAlliance() {
        return Alliance.RED;
    }

    @Override
    public void auto() {
        drive.setPoseEstimate(new Pose2d(6.75, -65.75, 0));

        //Pre-Setup
        driveTo(30.75, -65.75, Math.toRadians(45));

        while (opModeIsActive()) {
            //Align with wall
            driveToWall();

            //Drive through gap
            driveIntoWarehouse();

            //Intake / Don't hit wall
            boolean success = intake();
            //Park if nothing is in intake
            if (getRuntime() > 25) return;

            //Drive out through gap
            driveOutOfWarehouse();

            //Drive to and score in hub
            scoreInHub();
        }
    }

    private void driveToWall() {
        distCorrect.startRunning();

        lift.setTargetHeight(Lift.LOW_HEIGHT);

        H_ACC = Math.toRadians(5);
        POS_ACC = 2;

        driveTo(() -> {
                    if (deltaHeading(drive.getPoseEstimate().getHeading(), Math.toRadians(270)) > Math.toRadians(3))
                        return 6.5;
                    else
                        return drive.getPoseEstimate().getX();
                },
                () -> -70.0,
                () -> Math.toRadians(270)
        );

        drive.setWeightedDrivePower(new Pose2d(0, -0.5, 0));
        customWait(() -> (distCorrect.getSideWall() > 7.4));
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));

        drive.setPoseEstimate(new Pose2d(
                distCorrect.getSideWall(),
                drive.getPoseEstimate().getY(),
                Math.toRadians(270)));

        H_ACC = Math.toRadians(1);
        POS_ACC = 0.5;

        distCorrect.stopRunning();
    }

    private void driveIntoWarehouse() {
        distCorrect.startRunning();

        lift.setTargetHeight(Lift.GROUND_HEIGHT);
        intake.setIntakePower(-1);

        POS_ACC = 1.5;

        driveTo(() -> {
                    if (Math.abs(drive.getPoseEstimate().getY() + 105) > POS_ACC)
                        return drive.getPoseEstimate().getX() - 2;
                    else
                        return drive.getPoseEstimate().getX();
                },
                () -> -105.0,
                () -> Math.toRadians(268)
        , false);

        drive.setPoseEstimate(new Pose2d(distCorrect.correctPoseWithDist(), Math.toRadians(270)));

        POS_ACC = 0.5;

        distCorrect.stopRunning();
    }

    private boolean intake() {
        distCorrect.startRunning();

        lift.setTargetHeight(Lift.GROUND_HEIGHT);
        intake.setIntakePower(-1);

        POS_ACC = 6;
        H_ACC = Math.toRadians(5);

        drive.setWeightedDrivePower(new Pose2d(0.4, 0, 0.01));
        customWait(() -> (!intake.getFreightInIntake() && distCorrect.getFrontDistance() > 10));
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));

        POS_ACC = 0.5;
        H_ACC = Math.toRadians(1);

        distCorrect.stopRunning();

        if (intake.getFreightInIntake()) {
            intake.setIntakePower(-0.2);
            return true;
        }

        intake.setIntakePower(0);
        return false;
    }

    private void driveOutOfWarehouse() {
        distCorrect.startRunning();

        POS_ACC = 1;

        driveTo(() -> {
                    if (Math.abs(drive.getPoseEstimate().getY() + 70) > POS_ACC)
                        return drive.getPoseEstimate().getX() - 2;
                    else
                        return drive.getPoseEstimate().getX();
                },
                () -> {
                    if (distCorrect.getFrontDistance() < 30)
                        drive.setPoseEstimate(new Pose2d(distCorrect.correctPoseWithDist(), Math.toRadians(270)));

                    return -70.0;
                },
                () -> Math.toRadians(270)
        , false);

        drive.setPoseEstimate(new Pose2d(
                distCorrect.getSideWall(),
                drive.getPoseEstimate().getY(),
                Math.toRadians(270)));

        POS_ACC = 0.5;

        distCorrect.stopRunning();
    }

    private void scoreInHub() {
        lift.setTargetHeight(Lift.HIGH_HEIGHT);

        POS_ACC = 1;
        driveTo(35, -80, Math.toRadians(30));
        POS_ACC = 0.5;

        commands.outtake(intake);
    }
}
