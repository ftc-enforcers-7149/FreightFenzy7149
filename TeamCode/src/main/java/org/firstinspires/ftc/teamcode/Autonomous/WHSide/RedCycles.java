package org.firstinspires.ftc.teamcode.Autonomous.WHSide;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.Auto_V2_5;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Levels;

@Autonomous(name = "Red WH Cycles")
//@Disabled
public class RedCycles extends Auto_V2_5 {

    @Override
    protected Alliance getAlliance() {
        return Alliance.RED;
    }

    @Override
    public void auto() {
        drive.setPoseEstimate(new Pose2d(6.75, -78.25, 0));

        //Score pre-loaded
        lift.setTargetHeight(commands.detectBarcode(tseDetector));

        SPEED_MULT = 0.8;
        driveTo(31, -64, Math.toRadians(30));
        SPEED_MULT = 1;
        commands.outtake(intake, 1250);

        //Cycles
        int cycle = 0;

        while (opModeIsActive()) {
            //Align with wall
            driveToWall();

            //Drive through gap
            driveIntoWarehouse();

            //Intake / Don't hit wall
            intake(Math.max(20 - (cycle * 8), 12));
            //Park if running out of time
            if (getRuntime() > 26) {
                commands.setLiftHeight(lift, Levels.GROUND);
                return;
            }

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
                    if (Math.abs(deltaHeading(drive.getPoseEstimate().getHeading(), Math.toRadians(280))) > Math.toRadians(3))
                        return 7.5;
                    else {
                        lift.setTargetHeight(Levels.GROUND);
                        return drive.getPoseEstimate().getX();
                    }
                },
                () -> -77.0,
                () -> Math.toRadians(280)
        );

        long driveStartTime = System.currentTimeMillis();
        drive.setWeightedDrivePower(new Pose2d(0.005, -0.5, 0));
        customWait(() -> (distCorrect.getSideWall() > 8) && System.currentTimeMillis() < driveStartTime + 1000);
        waitForTime(300);
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));

        drive.setPoseEstimate(new Pose2d(
                distCorrect.getSideWall(),
                -77,
                Math.toRadians(270)));

        H_ACC = Math.toRadians(1);
        POS_ACC = 0.5;

        distCorrect.stopRunning();
    }

    private void driveIntoWarehouse() {
        distCorrect.startRunning();
        intake.startScanningIntake();

        lift.setTargetHeight(Levels.GROUND);
        intake.setIntakePower(-1);

        POS_ACC = 2;
        H_ACC = Math.toRadians(20);

        drive.setWeightedDrivePower(new Pose2d(0.5, -0.4, 0));
        long driveStartTime = System.currentTimeMillis();
        customWait(() -> !intake.getFreightInIntake() &&
                System.currentTimeMillis() < driveStartTime + 750);
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));

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

        lift.setTargetHeight(Levels.GROUND);
        intake.setIntakePower(-1);

        POS_ACC = 6;
        H_ACC = Math.toRadians(2);
        MIN_TURN = 0.3;

        long driveStartTime = System.currentTimeMillis();
        drive.setWeightedDrivePower(new Pose2d(0.3, -0.1, 0));
        customWait(() -> (!intake.getFreightInIntake() &&
                distCorrect.getFrontDistance() > distanceFromWall &&
                System.currentTimeMillis() < driveStartTime + 2000));
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));

        POS_ACC = 0.5;
        H_ACC = Math.toRadians(1);
        MIN_TURN = 0.2;

        long correctStartTime = System.currentTimeMillis();
        customWait(() -> distCorrect.getFrontDistance() > 50 && System.currentTimeMillis() < correctStartTime + 2000);

        if (distCorrect.getFrontDistance() < 50) {
            drive.setPoseEstimate(distCorrect.correctPoseWithDist(drive.getPoseEstimate().getHeading()));
        }
        else
            drive.setPoseEstimate(new Vector2d(distCorrect.getSideWall(), -144));

        intake.stopScanningIntake();
        distCorrect.stopRunning();

        intake.setIntakePower(-0.2);
    }

    private void driveOutOfWarehouse() {
        distCorrect.startRunning();

        POS_ACC = 2;
        SLOW_DIST = 2;
        SPEED_MULT = 0.5;

        driveTo(() -> {
                    if (Math.abs(drive.getPoseEstimate().getY() + 76) > POS_ACC * 3)
                        return drive.getPoseEstimate().getX() - 10;
                    else
                        return drive.getPoseEstimate().getX();
                },
                () -> {
                    if (distCorrect.getSideWall() > 15)
                        return -85.0;
                    else {
                        SLOW_DIST = 15;
                        SPEED_MULT = 0.8;

                        return -76.0;
                    }
                },
                () -> {
                    if (distCorrect.getFrontDistance() < 40 &&
                            distCorrect.getSideWall() < 20) {
                        double robotHeading = drive.getPoseEstimate().getHeading();
                        drive.setPoseEstimate(new Pose2d(distCorrect.correctPoseWithDist(robotHeading), robotHeading));
                    }

                    if (distCorrect.getSideWall() > 15)
                        return Math.toRadians(271);
                    else
                        return Math.toRadians(270);
                }
        , 2000);

        drive.setPoseEstimate(new Pose2d(
                distCorrect.getSideWall(),
                drive.getPoseEstimate().getY(),
                Math.toRadians(270)));

        POS_ACC = 0.5;
        SPEED_MULT = 1;

        distCorrect.stopRunning();
    }

    private void scoreInHub() {
        lift.setTargetHeight(Levels.HIGH);

        POS_ACC = 1;
        H_ACC = Math.toRadians(4);
        driveTo(35, -70, Math.toRadians(25));
        POS_ACC = 0.5;
        H_ACC = Math.toRadians(1);

        commands.outtake(intake);
    }
}
