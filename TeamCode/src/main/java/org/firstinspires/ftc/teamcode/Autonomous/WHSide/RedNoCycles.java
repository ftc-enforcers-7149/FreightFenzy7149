package org.firstinspires.ftc.teamcode.Autonomous.WHSide;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.Auto_V2;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Lift;

import java.util.function.Supplier;

@Autonomous(name = "Red No Cycles")
@Disabled
public class RedNoCycles extends Auto_V2 {

    @Override
    protected Alliance getAlliance() {
        return Alliance.RED;
    }

    @Override
    public void auto() {
        drive.setPoseEstimate(new Pose2d(6.75, -78.25, 0));

        //Score pre-loaded
        intake.setIntakePower(-0.2);

        switch (commands.detectBarcode(tseDetector)) {
            case LOW:
                lift.setTargetHeight(Lift.LOW_HEIGHT);
                break;
            case MIDDLE:
                lift.setTargetHeight(Lift.MIDDLE_HEIGHT);
                break;
            case HIGH:
            default:
                lift.setTargetHeight(Lift.HIGH_HEIGHT);
                break;
        }

        driveTo(35, -77, Math.toRadians(30));
        commands.outtake(intake);

        //Cycles
        int cycle = 0;

        while (opModeIsActive()) {
            //Align with wall
            driveToWall();

            //Drive through gap
            driveIntoWarehouse();

            //Intake / Don't hit wall
            intake(Math.min(28 - (cycle * 2), 12));
            //Park if running out of time
            if (getRuntime() > 25) return;

            //Drive out through gap
            driveOutOfWarehouse();

            //Drive to and score in hub
            scoreInHub();

            cycle++;
        }
    }

    private void driveToWall() {
        distCorrect.startRunning();

        H_ACC = Math.toRadians(5);
        POS_ACC = 2;

        driveTo(() -> {
                    if (deltaHeading(drive.getPoseEstimate().getHeading(), Math.toRadians(275)) > Math.toRadians(3))
                        return 7.5;
                    else {
                        lift.setTargetHeight(Lift.LOW_HEIGHT);
                        return drive.getPoseEstimate().getX();
                    }
                },
                () -> -77.0,
                () -> Math.toRadians(275)
        );

        drive.setWeightedDrivePower(new Pose2d(0, -0.5, 0));
        customWait(() -> (distCorrect.getSideWall() > 8.2));
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
        intake.startScanningIntake();

        lift.setTargetHeight(Lift.GROUND_HEIGHT);
        intake.setIntakePower(-1);

        POS_ACC = 2;
        H_ACC = Math.toRadians(20);

        long startTime = System.currentTimeMillis();
        Supplier<Boolean> inTime = () -> System.currentTimeMillis() < startTime + 1500;

        driveTo(() -> {
                    if (Math.abs(drive.getPoseEstimate().getY() + 80) > POS_ACC * 3 &&
                            inTime.get() && !intake.getFreightInIntake())
                        return drive.getPoseEstimate().getX() - 16;
                    else
                        return drive.getPoseEstimate().getX();
                },
                () -> {
                        if (inTime.get() && !intake.getFreightInIntake())
                            return -80.0;
                        else
                            return drive.getPoseEstimate().getY();
                },
                () -> {
                    if (inTime.get() && !intake.getFreightInIntake())
                        return Math.toRadians(268);
                    else
                        return drive.getPoseEstimate().getHeading();
                }
        );

        drive.setPoseEstimate(new Pose2d(
                distCorrect.getSideWall(),
                drive.getPoseEstimate().getY(),
                Math.toRadians(270)));

        POS_ACC = 0.5;
        H_ACC = Math.toRadians(1);

        intake.stopScanningIntake();
        distCorrect.stopRunning();
    }

    private void intake(double distanceFromWall) {
        distCorrect.startRunning();
        intake.startScanningIntake();

        lift.setTargetHeight(Lift.GROUND_HEIGHT);
        intake.setIntakePower(-1);

        POS_ACC = 6;
        H_ACC = Math.toRadians(1);

        drive.setWeightedDrivePower(new Pose2d(0.5, 0, 0));
        customWait(() -> (!intake.getFreightInIntake() && distCorrect.getFrontDistance() > distanceFromWall));
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));

        driveTo(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), Math.toRadians(270));

        POS_ACC = 0.5;
        H_ACC = Math.toRadians(1);

        double robotHeading = drive.getPoseEstimate().getHeading();
        drive.setPoseEstimate(new Pose2d(distCorrect.correctPoseWithDist(robotHeading), robotHeading));

        intake.stopScanningIntake();
        distCorrect.stopRunning();

        intake.setIntakePower(-0.2);
    }

    private void driveOutOfWarehouse() {
        distCorrect.startRunning();

        POS_ACC = 2;

        driveTo(() -> {
                    if (Math.abs(drive.getPoseEstimate().getY() + 70) > POS_ACC * 3)
                        return drive.getPoseEstimate().getX() - 16;
                    else
                        return drive.getPoseEstimate().getX();
                },
                () -> {
                    drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(),
                                                     drive.getPoseEstimate().getY(),
                                                     Math.toRadians(270)));

                    if (distCorrect.getSideWall() > 15)
                        return -95.0;
                    else
                        return -70.0;
                },
                () -> Math.toRadians(270)
        );

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
        H_ACC = Math.toRadians(4);
        driveTo(35, -74, Math.toRadians(20));
        POS_ACC = 0.5;
        H_ACC = Math.toRadians(1);

        commands.outtake(intake);
    }
}