package org.firstinspires.ftc.teamcode.Autonomous.WHSide;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.Auto_V2_5;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.MotorIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Levels;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

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

        SLOW_DIST = 25;
        SPEED_MULT = 0.9;
        Lift.pidCoeffs = new PIDCoefficients(0.007, 0, 0.0002);

        //Score pre-loaded
        lift.setTargetHeight(commands.detectBarcode(tseDetector));

        H_ACC = Math.toRadians(4);
        driveTo(31, -68, Math.toRadians(30));
        H_ACC = Math.toRadians(1);
        commands.outtake(intake, lift);

        SLOW_DIST = 20;
        Lift.pidCoeffs = new PIDCoefficients(0.008, 0, 0.0002);

        //Cycles
        int cycle = 0;

        boolean inWarehouse = false;

        while (opModeIsActive() && cycle < 4) {
            if (intake.getFreightInIntake()) {
                intake.setLatch(MotorIntake.LatchPosition.OPEN);
                intake.setIntakePower(-1);
            }

            //Align with wall
            driveToWall();

            if (!intake.getFreightInIntake())
                intake.setIntakePower(0);

            //Drive through gap
            driveIntoWarehouse();
            inWarehouse = true;

            //Park if running out of time
            if (getRuntime() >= 29)
                break;

            //Intake / Don't hit wall
            if (!intake.getFreightInIntake())
                intake(Math.max(22 - (cycle * 6), 8));
            else
                drive.setMotorPowers(0, 0, 0, 0);

            //Drive out through gap
            driveOutOfWarehouse();
            inWarehouse = false;

            //Drive to and score in hub
            scoreInHub(-78);

            cycle++;
        }
        intake.setLatch(MotorIntake.LatchPosition.OPEN);
        intake.setPaddle(MotorIntake.PaddlePosition.BACK);

        if (!inWarehouse) {
            driveToWall();
            driveIntoWarehouse();
            intake(40);
        }
        lift.setTargetHeight(Levels.GROUND);
        intake.setLatch(MotorIntake.LatchPosition.CLOSED);
        intake.setIntakePower(-0.3);
        SPEED_MULT = 1.0;
        SLOW_DIST = 5;
        driveTo(drive.getPoseEstimate().getX() + 2, drive.getPoseEstimate().getY() - 6, 0);
        SLOW_DIST = 20;
        intake.setIntakePower(0);
    }

    private void driveToWall() {
        distCorrect.startRunning();

        H_ACC = Math.toRadians(5);
        POS_ACC = 2;
        SLOW_DIST = 1;
        SPEED_MULT = 1.0;

        lift.setPower(-0.01); //Start moving the lift down

        driveTo(() -> {
                    if (Math.abs(deltaHeading(drive.getPoseEstimate().getHeading(), Math.toRadians(275))) > Math.toRadians(3))
                        return 12.0;
                    else {
                        lift.setTargetHeight(Levels.GROUND);
                        return drive.getPoseEstimate().getX();
                    }
                },
                () -> -80.0,
                () -> Math.toRadians(275)
        , false);

        long driveStartTime = System.currentTimeMillis();
        drive.setWeightedDrivePower(new Pose2d(0.02, -0.7, 0));
        customWait(() -> (distCorrect.getSideWall() > 8.5) && System.currentTimeMillis() < driveStartTime + 1000);
        waitForTime(75);
        //drive.setWeightedDrivePower(new Pose2d(0, 0, 0));

        drive.setPoseEstimate(new Pose2d(
                distCorrect.getSideWall(),
                -77,
                Math.toRadians(270)));

        H_ACC = Math.toRadians(1);
        POS_ACC = 1;
        SLOW_DIST = 20;
        SPEED_MULT = 0.9;

        distCorrect.stopRunning();
    }

    private void driveIntoWarehouse() {
        distCorrect.startRunning();
        //intake.startScanningIntake();

        lift.setTargetHeight(Levels.GROUND);
        intake.setIntakePower(1);

        POS_ACC = 2;
        H_ACC = Math.toRadians(20);

        drive.setWeightedDrivePower(new Pose2d(0.9, -0.35, 0));
        long driveStartTime = System.currentTimeMillis();

        customWait(() -> System.currentTimeMillis() < driveStartTime + 350 &&
                distCorrect.getFrontDistance() > 50);
        //drive.setWeightedDrivePower(new Pose2d(0, 0, 0));

        drive.setPoseEstimate(new Pose2d(
                distCorrect.getSideWall(),
                drive.getPoseEstimate().getY(),
                Math.toRadians(270)));

        POS_ACC = 1;
        H_ACC = Math.toRadians(1);

        //intake.stopScanningIntake();
        distCorrect.stopRunning();
    }

    private void intake(double distanceFromWall) {
        distCorrect.startRunning();
        intake.startScanningIntake();

        lift.setTargetHeight(Levels.GROUND);
        intake.setIntakePower(1);

        long driveStartTime = System.currentTimeMillis();
        drive.setWeightedDrivePower(new Pose2d(0.6, -0.3, 0));
        customWait(() -> {
            if (!intake.getFreightInIntake() && distCorrect.getFrontDistance() < 50)
                drive.setWeightedDrivePower(new Pose2d(Math.pow(distCorrect.getFrontDistance(), 2) * 0.00032, -0.1, 0));

            return ((!intake.getFreightInIntake() &&
                    distCorrect.getFrontDistance() > distanceFromWall) ||
                    distCorrect.getFrontDistance() > 30) &&
                    System.currentTimeMillis() < driveStartTime + 1000;
        });
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));

        long correctStartTime = System.currentTimeMillis();
        customWait(() -> distCorrect.getFrontDistance() > 50 && System.currentTimeMillis() < correctStartTime + 1000);

        if (distCorrect.getFrontDistance() < 50)
            drive.setPoseEstimate(distCorrect.correctPoseWithDist(drive.getPoseEstimate().getHeading()));
        else
            drive.setPoseEstimate(new Vector2d(distCorrect.getSideWall(), -144));

        intake.stopScanningIntake();
        //intake.setLatch(MotorIntake.LatchPosition.CLOSED);
        //intake.setIntakePower(0);
        distCorrect.stopRunning();
    }

    private void driveOutOfWarehouse() {
        distCorrect.startRunning();

        POS_ACC = 3;
        SLOW_DIST = 2;
        SPEED_MULT = 0.9;

        if (drive.getPoseEstimate().getY() < -85) {
            drive.setWeightedDrivePower(new Pose2d(-0.03, -0.5, 0));
            waitForTime(150);
        }

        AtomicReference<Double> lastFrontReading = new AtomicReference<>(distCorrect.getFrontDistance());

        AtomicBoolean throughGap = new AtomicBoolean(false);
        AtomicBoolean spitOut = new AtomicBoolean(false);

        driveTo(() -> {
                    if (!throughGap.get())
                        return drive.getPoseEstimate().getX() - 10;
                    else
                        return drive.getPoseEstimate().getX();
                },
                () -> {
                    if (distCorrect.getSideWall() > 15)
                        return -100.0;
                    else {
                        SLOW_DIST = 10;

                        return -85.0;
                    }
                },
                () -> {
                    if (distCorrect.getFrontDistance() < 45 &&
                        distCorrect.getFrontDistance() > lastFrontReading.get() + 2) {
                        double robotHeading = drive.getPoseEstimate().getHeading();
                        drive.setPoseEstimate(new Pose2d(distCorrect.correctPoseWithDist(robotHeading), robotHeading));
                        lastFrontReading.set(distCorrect.getFrontDistance());
                    }
                    else if (distCorrect.getFrontDistance() >= 50 && !throughGap.get()) {
                        double robotHeading = drive.getPoseEstimate().getHeading();
                        drive.setPoseEstimate(new Pose2d(7, -96, robotHeading));

                        throughGap.set(true);
                    }

                    if (distCorrect.getFrontDistance() > 30 && !spitOut.get()) {
                        intake.spitOutTwo();
                        spitOut.set(true);
                    }

                    if (distCorrect.getSideWall() > 15)
                        return Math.toRadians(271);
                    else
                        return Math.toRadians(270);
                }
        , 1500);

        drive.setPoseEstimate(new Pose2d(
                distCorrect.getSideWall(),
                drive.getPoseEstimate().getY(),
                Math.toRadians(270)));

        POS_ACC = 1;
        SPEED_MULT = 0.9;
        SLOW_DIST = 20;

        intake.cancelSpitOut();
        intake.setLatch(MotorIntake.LatchPosition.CLOSED);
        intake.setIntakePower(0);

        distCorrect.stopRunning();
    }

    private void scoreInHub() {
        lift.setTargetHeight(Levels.HIGH);

        H_ACC = Math.toRadians(4);
        driveTo(31.75, -62, Math.toRadians(25), 1500);
        H_ACC = Math.toRadians(1);

        commands.outtake(intake, lift);
    }

    private void scoreInHub(double yPos) {
        lift.setTargetHeight(Levels.HIGH);

        SLOW_DIST = 25;
        H_ACC = Math.toRadians(4);
        driveTo(32.5, yPos, Math.toRadians(25), 1500);
        H_ACC = Math.toRadians(1);
        SLOW_DIST = 20;

        commands.outtake(intake, lift);
    }
}
