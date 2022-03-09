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

import static org.firstinspires.ftc.teamcode.GlobalData.*;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

@Autonomous(name = "Blue WH Cycles")
//@Disabled
public class BlueCycles extends Auto_V2_5 {

    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }

    @Override
    public void auto() {
        drive.setPoseEstimate(new Pose2d(6.75, 78.25, 0));

        SLOW_DIST = 25;
        SPEED_MULT = 0.9;
        Lift.pidCoeffs = new PIDCoefficients(0.0045, 0, 0.0001);
        lift.initPID();

        //Score pre-loaded
        lift.setTargetHeight(commands.detectBarcode(tseDetector));
        waitForTime(100);

        H_ACC = Math.toRadians(4);
        SPEED_MULT = 0.8;
        driveTo(31.75, 71, Math.toRadians(330));
        H_ACC = Math.toRadians(1);
        SPEED_MULT = 0.9;
        commands.outtake(intake, lift);

        SLOW_DIST = 20;
        Lift.pidCoeffs = new PIDCoefficients(0.006, 0, 0.00015);
        lift.initPID();

        //Cycles
        int cycle = 0;

        boolean inWarehouse = false;

        while (opModeIsActive() && cycle < 3) {
            if (intake.getFreightInIntake()) {
                intake.setLatch(MotorIntake.LatchPosition.OPEN);
                intake.setIntakePower(-1);
            }

            //Align with wall
            if (cycle == 0)
                driveToWall(75);
            else
                driveToWall(75);

            if (!intake.getFreightInIntake())
                intake.setIntakePower(0);

            //Drive through gap
            driveIntoWarehouse();
            inWarehouse = true;

            //Park if running out of time
            if (getRuntime() >= 29)
                break;

            //Intake / Don't hit wall
            intake(Math.max(22 - (cycle * 6), 5));

            //Drive out through gap
            driveOutOfWarehouse();
            inWarehouse = false;

            //Drive to and score in hub
            scoreInHub(78);

            cycle++;
        }
        intake.setLatch(MotorIntake.LatchPosition.OPEN);
        intake.setPaddle(MotorIntake.PaddlePosition.BACK);

        if (!inWarehouse) {
            driveToWall(78);
            driveIntoWarehouse();
            intake(50);
        }
        lift.setTargetHeight(Levels.GROUND);
        intake.setLatch(MotorIntake.LatchPosition.CLOSED);
        intake.setIntakePower(-0.3);
        SPEED_MULT = 1.0;
        SLOW_DIST = 5;
        driveTo(drive.getPoseEstimate().getX() + 2, drive.getPoseEstimate().getY() + 6, 0);
        SLOW_DIST = 20;
        intake.setIntakePower(0);
    }

    private void driveToWall(double endPos) {
        distCorrect.startSideSensor();

        H_ACC = Math.toRadians(5);
        POS_ACC = 2;
        SLOW_DIST = 10;
        SPEED_MULT = 0.7;

        lift.setPower(-0.01); //Start moving the lift down

        boolean timeOut = driveTo(() -> {
                    if (Math.abs(deltaHeading(drive.getPoseEstimate().getHeading(), Math.toRadians(85))) > Math.toRadians(3))
                        return 20.0;
                    else {
                        lift.setTargetHeight(Levels.GROUND);
                        return drive.getPoseEstimate().getX();
                    }
                },
                () -> endPos,
                () -> Math.toRadians(85)
        , 2000, true);

        long driveStartTime = System.currentTimeMillis();
        drive.setWeightedDrivePower(new Pose2d(0.01, 0.4, 0));
        customWait(() -> distCorrect.getSideWall() > 8.5 && System.currentTimeMillis() < driveStartTime + 1150);
        waitForTime(100);
        //drive.setWeightedDrivePower(new Pose2d(0, 0, 0));

        if (!timeOut) {
            drive.setPoseEstimate(new Pose2d(
                    distCorrect.getSideWall(),
                    endPos,
                    Math.toRadians(90)));
        }
        else {
            drive.setPoseEstimate(new Pose2d(
                    8.5,
                    endPos,
                    Math.toRadians(90)));
        }

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

        drive.setWeightedDrivePower(new Pose2d(0.5, 0.35, 0));

        long driveStartTime = System.currentTimeMillis();
        customWait(() -> System.currentTimeMillis() < driveStartTime + 450 &&
                distCorrect.getFrontDistance() > 50);
        //drive.setWeightedDrivePower(new Pose2d(0, 0, 0));

        drive.setPoseEstimate(new Pose2d(
                distCorrect.getSideWall(),
                drive.getPoseEstimate().getY(),
                Math.toRadians(90)));

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
        drive.setWeightedDrivePower(new Pose2d(0.4, 0.3, 0));
        customWait(() -> {
            if (distCorrect.getFrontDistance() < 50)
                drive.setWeightedDrivePower(new Pose2d(
                        Math.min(Math.pow(distCorrect.getFrontDistance(), 2) * 0.0002, 0.4), //Slow down as approaches wall
                        -distanceFromWall * (0.2 / 22), //Drive away from wall
                        0));

            return ((!intake.getFreightInIntake() &&
                    distCorrect.getFrontDistance() > distanceFromWall) ||
                    distCorrect.getFrontDistance() > 30) &&
                    System.currentTimeMillis() < driveStartTime + 2000;
        });
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));

        boolean wait = false;
        if (distCorrect.getFrontDistance() > 500) {
            distCorrect.stopRunning();
            distCorrect.setSensorF(hardwareMap, "distF");
            wait = true;
        }
        else if(distCorrect.getSideWall() > 500) {
            distCorrect.stopRunning();
            distCorrect.setSensorL(hardwareMap, "distL");
            wait = true;
        }

        long sensorFixStartTime = System.currentTimeMillis();
        if (wait)
            while (opModeIsActive() && System.currentTimeMillis() < sensorFixStartTime + 2000) {
                telemetry.addLine("Attempting to fix sensors");
                telemetry.update();
            }

        long correctStartTime = System.currentTimeMillis();
        customWait(() -> distCorrect.getFrontDistance() > 50 && System.currentTimeMillis() < correctStartTime + 1000);

        if (distCorrect.getFrontDistance() < 50)
            drive.setPoseEstimate(distCorrect.correctPoseWithDist(drive.getPoseEstimate().getHeading()));
        else
            requestOpModeStop();

        intake.stopScanningIntake();
        //intake.setLatch(MotorIntake.LatchPosition.CLOSED);
        //intake.setIntakePower(0);
        distCorrect.stopRunning();
    }

    private void driveOutOfWarehouse() {
        distCorrect.startRunning();

        POS_ACC = 3;
        SLOW_DIST = 2;
        SPEED_MULT = 0.7;

        drive.setWeightedDrivePower(new Pose2d(-0.04, 0.5, 0));
        waitForTime(175);

        long correctStartTime = System.currentTimeMillis();
        customWait(() -> distCorrect.getFrontDistance() > 50 && System.currentTimeMillis() < correctStartTime + 1000);

        if (distCorrect.getFrontDistance() < 50)
            drive.setPoseEstimate(distCorrect.correctPoseWithDist(drive.getPoseEstimate().getHeading()));

        AtomicReference<Double> lastFrontReading = new AtomicReference<>(distCorrect.getFrontDistance());

        AtomicBoolean throughGap = new AtomicBoolean(false);
        AtomicBoolean spitOut = new AtomicBoolean(false);

        boolean timeOut = driveTo(() -> {
                    if (!throughGap.get())
                        return drive.getPoseEstimate().getX() - 15;
                    else
                        return drive.getPoseEstimate().getX();
                },
                () -> {
                    if (distCorrect.getSideWall() > 15 && !throughGap.get())
                        return 90.0;
                    else {
                        SLOW_DIST = 10;

                        return 85.0;
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
                        drive.setPoseEstimate(new Pose2d(7, 96, robotHeading));

                        throughGap.set(true);
                    }

                    if (distCorrect.getFrontDistance() > 45 && !spitOut.get()) {
                        intake.spitOutTwo();
                        spitOut.set(true);
                    }

                    if (distCorrect.getSideWall() > 15)
                        return Math.toRadians(89);
                    else
                        return Math.toRadians(90);
                }
        , 2000);

        if (!timeOut) {
            drive.setPoseEstimate(new Pose2d(
                    distCorrect.getSideWall(),
                    drive.getPoseEstimate().getY(),
                    Math.toRadians(90)));
        }
        else {
            drive.setPoseEstimate(new Pose2d(
                    8.5,
                    drive.getPoseEstimate().getY(),
                    Math.toRadians(90)));
        }

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
        driveTo(36, 62, Math.toRadians(330), 1500);
        H_ACC = Math.toRadians(1);

        commands.outtake(intake, lift);
    }

    private void scoreInHub(double yPos) {
        lift.setTargetHeight(Levels.HIGH);

        SPEED_MULT = 0.7;
        SLOW_DIST = 25;
        H_ACC = Math.toRadians(4);
        driveTo(36, yPos, Math.toRadians(330), 1500);
        H_ACC = Math.toRadians(1);
        SLOW_DIST = 20;
        SPEED_MULT = 0.9;

        commands.outtake(intake, lift);
    }
}
