package org.firstinspires.ftc.teamcode.Autonomous.WHSide;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.Auto_V2;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Lift;

import java.util.function.Supplier;

@Autonomous(name = "Blue WH Cycles")
//@Disabled
public class    BlueCycles extends Auto_V2 {

    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }

    @Override
    public void auto() {
        drive.setPoseEstimate(new Pose2d(6.75, 78.25, 0));

        //Score pre-loaded
        switch (commands.detectBarcode(tseDetector)) {
            case LOW:
                lift.setTargetHeight(Lift.LOW_HEIGHT + 1);
                break;
            case MIDDLE:
                lift.setTargetHeight(Lift.MIDDLE_HEIGHT);
                break;
            case HIGH:
            default:
                lift.setTargetHeight(Lift.HIGH_HEIGHT);
                break;
        }

        SPEED_MULT = 0.8;
        driveTo(32, 68, Math.toRadians(330));
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
                commands.setLiftHeight(lift, Lift.GROUND_HEIGHT);
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
                    if (Math.abs(deltaHeading(drive.getPoseEstimate().getHeading(), Math.toRadians(80))) > Math.toRadians(3))
                        return 7.5;
                    else {
                        lift.setTargetHeight(Lift.LOW_HEIGHT);
                        return drive.getPoseEstimate().getX();
                    }
                },
                () -> 77.0,
                () -> Math.toRadians(80)
        );

        long driveStartTime = System.currentTimeMillis();
        drive.setWeightedDrivePower(new Pose2d(0.005, 0.5, 0));
        customWait(() -> (distCorrect.getSideWall() > 8.2) && System.currentTimeMillis() < driveStartTime + 1000);
        waitForTime(300);
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));

        drive.setPoseEstimate(new Pose2d(
                distCorrect.getSideWall(),
                -77,
                Math.toRadians(90)));

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

        drive.setWeightedDrivePower(new Pose2d(0.5, 0.4, 0));
        long driveStartTime = System.currentTimeMillis();
        customWait(() -> !intake.getFreightInIntake() &&
                System.currentTimeMillis() < driveStartTime + 750);
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));

        drive.setPoseEstimate(new Pose2d(
                distCorrect.getSideWall(),
                drive.getPoseEstimate().getY(),
                Math.toRadians(90)));

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
        H_ACC = Math.toRadians(2);
        MIN_TURN = 0.3;

        long driveStartTime = System.currentTimeMillis();
        drive.setWeightedDrivePower(new Pose2d(0.3, 0.1, 0));
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
            drive.setPoseEstimate(new Vector2d(distCorrect.getSideWall(), 144));

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
                    if (Math.abs(drive.getPoseEstimate().getY() - 74) > POS_ACC * 3)
                        return drive.getPoseEstimate().getX() - 10;
                    else
                        return drive.getPoseEstimate().getX();
                },
                () -> {
                    if (distCorrect.getSideWall() > 15)
                        return 85.0;
                    else {
                        SLOW_DIST = 15;
                        SPEED_MULT = 0.8;

                        return 74.0;
                    }
                },
                () -> {
                    if (distCorrect.getFrontDistance() < 40 &&
                        distCorrect.getSideWall() < 20) {
                        double robotHeading = drive.getPoseEstimate().getHeading();
                        drive.setPoseEstimate(new Pose2d(distCorrect.correctPoseWithDist(robotHeading), robotHeading));
                    }

                    if (distCorrect.getSideWall() > 15)
                        return Math.toRadians(89);
                    else
                        return Math.toRadians(90);
                }
        , 2000);

        drive.setPoseEstimate(new Pose2d(
                distCorrect.getSideWall(),
                drive.getPoseEstimate().getY(),
                Math.toRadians(90)));

        POS_ACC = 0.5;
        SPEED_MULT = 1;

        distCorrect.stopRunning();
    }

    private void scoreInHub() {
        lift.setTargetHeight(Lift.HIGH_HEIGHT);

        POS_ACC = 1;
        H_ACC = Math.toRadians(4);
        driveTo(35, 70, Math.toRadians(335));
        POS_ACC = 0.5;
        H_ACC = Math.toRadians(1);

        commands.outtake(intake);
    }
}
