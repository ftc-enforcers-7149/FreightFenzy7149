package org.firstinspires.ftc.teamcode.Autonomous.WHSide;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.Auto_V2_5;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.MotorIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Levels;

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

        //Score pre-loaded
        lift.setTargetHeight(commands.detectBarcode(tseDetector));

        driveTo(33, 68, Math.toRadians(330));
        commands.outtake(intake, lift);

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
                intake(Math.max(26 - (cycle * 5), 12));

            //Drive out through gap
            driveOutOfWarehouse();
            inWarehouse = false;

            //Drive to and score in hub
            if (cycle < 2)
                scoreInHub(62);
            else
                scoreInHub(68);

            cycle++;
        }
        intake.setLatch(MotorIntake.LatchPosition.OPEN);
        intake.setPaddle(MotorIntake.PaddlePosition.BACK);

        if (!inWarehouse) {
            driveToWall();
            driveIntoWarehouse();
            intake(30);
        }
        lift.setTargetHeight(Levels.GROUND);
        intake.setLatch(MotorIntake.LatchPosition.CLOSED);
        intake.setIntakePower(-0.3);
        SPEED_MULT = 1.0;
        driveTo(drive.getPoseEstimate().getX() + 2, drive.getPoseEstimate().getY() + 6, 0);
        intake.setIntakePower(0);
    }

    private void driveToWall() {
        distCorrect.startRunning();

        H_ACC = Math.toRadians(5);
        POS_ACC = 2;
        SLOW_DIST = 10;
        SPEED_MULT = 1.0;

        lift.setPower(-0.01); //Start moving the lift down

        driveTo(() -> {
                    if (Math.abs(deltaHeading(drive.getPoseEstimate().getHeading(), Math.toRadians(80))) > Math.toRadians(3))
                        return 8.5;
                    else {
                        lift.setTargetHeight(Levels.GROUND);
                        return drive.getPoseEstimate().getX();
                    }
                },
                () -> 77.0,
                () -> Math.toRadians(80)
        );

        long driveStartTime = System.currentTimeMillis();
        drive.setWeightedDrivePower(new Pose2d(0, 0.5, 0));
        customWait(() -> distCorrect.getSideWall() > 8.3 && System.currentTimeMillis() < driveStartTime + 1000);
        waitForTime(125);
        //drive.setWeightedDrivePower(new Pose2d(0, 0, 0));

        drive.setPoseEstimate(new Pose2d(
                distCorrect.getSideWall(),
                -77,
                Math.toRadians(90)));

        H_ACC = Math.toRadians(1);
        POS_ACC = 1;
        SLOW_DIST = 25;
        SPEED_MULT = 0.9;

        distCorrect.stopRunning();
    }

    private void driveIntoWarehouse() {
        distCorrect.startRunning();
        intake.startScanningIntake();

        lift.setTargetHeight(Levels.GROUND);
        intake.setIntakePower(1);

        POS_ACC = 2;
        H_ACC = Math.toRadians(20);

        drive.setWeightedDrivePower(new Pose2d(0.7, 0.4, 0));
        long driveStartTime = System.currentTimeMillis();

        customWait(() -> !intake.getFreightInIntake() &&
                System.currentTimeMillis() < driveStartTime + 500);
        //drive.setWeightedDrivePower(new Pose2d(0, 0, 0));

        drive.setPoseEstimate(new Pose2d(
                distCorrect.getSideWall(),
                drive.getPoseEstimate().getY(),
                Math.toRadians(90)));

        POS_ACC = 1;
        H_ACC = Math.toRadians(1);

        intake.stopScanningIntake();
        distCorrect.stopRunning();
    }

    private void intake(double distanceFromWall) {
        distCorrect.startRunning();
        intake.startScanningIntake();

        lift.setTargetHeight(Levels.GROUND);
        intake.setIntakePower(1);

        long driveStartTime = System.currentTimeMillis();
        drive.setWeightedDrivePower(new Pose2d(0.5, 0.3, 0));
        customWait(() -> {
            if (!intake.getFreightInIntake() && distCorrect.getFrontDistance() < 40)
                drive.setWeightedDrivePower(new Pose2d(Math.pow(distCorrect.getFrontDistance(), 2) * 0.0003, 0.3, 0));

            return !intake.getFreightInIntake() &&
                    distCorrect.getFrontDistance() > distanceFromWall &&
                    System.currentTimeMillis() < driveStartTime + 2000;
        });
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));

        long correctStartTime = System.currentTimeMillis();
        customWait(() -> distCorrect.getFrontDistance() > 50 && System.currentTimeMillis() < correctStartTime + 2000);

        if (distCorrect.getFrontDistance() < 50)
            drive.setPoseEstimate(distCorrect.correctPoseWithDist(drive.getPoseEstimate().getHeading()));
        else
            drive.setPoseEstimate(new Vector2d(distCorrect.getSideWall(), 144));

        intake.stopScanningIntake();
        distCorrect.stopRunning();
    }

    private void driveOutOfWarehouse() {
        distCorrect.startRunning();

        POS_ACC = 3;
        SLOW_DIST = 2;
        SPEED_MULT = 0.6;

        AtomicReference<Double> lastFrontReading = new AtomicReference<>(distCorrect.getFrontDistance());
        AtomicReference<Double> lastSideReading = new AtomicReference<>(distCorrect.getSideWall());

        driveTo(() -> {
                    if (Math.abs(drive.getPoseEstimate().getY() - 60) > POS_ACC * 3) {
                        intake.setIntakePower(-0.5);
                        intake.setLatch(MotorIntake.LatchPosition.CLOSED);
                        return drive.getPoseEstimate().getX() - 1;
                    }
                    else
                        return drive.getPoseEstimate().getX();
                },
                () -> {
                    if (distCorrect.getSideWall() > 15)
                        return 85.0;
                    else {
                        SLOW_DIST = 10;
                        SPEED_MULT = 0.75;

                        return 60.0;
                    }
                },
                () -> {
                    if (distCorrect.getFrontDistance() < 40 &&
                        distCorrect.getSideWall() < 20 &&
                        distCorrect.getFrontDistance() > lastFrontReading.get() + 2 &&
                        distCorrect.getSideWall() < lastSideReading.get() - 2) {
                        double robotHeading = drive.getPoseEstimate().getHeading();
                        drive.setPoseEstimate(new Pose2d(distCorrect.correctPoseWithDist(robotHeading), robotHeading));
                        lastFrontReading.set(distCorrect.getFrontDistance());
                        lastSideReading.set(distCorrect.getSideWall());
                    }

                    if (distCorrect.getSideWall() > 15)
                        return Math.toRadians(89);
                    else
                        return Math.toRadians(90);
                }
        , 1500);

        drive.setPoseEstimate(new Pose2d(
                distCorrect.getSideWall(),
                drive.getPoseEstimate().getY(),
                Math.toRadians(90)));

        POS_ACC = 1;
        SPEED_MULT = 0.9;
        SLOW_DIST = 25;

        intake.setIntakePower(0);

        distCorrect.stopRunning();
    }

    private void scoreInHub() {
        lift.setTargetHeight(Levels.HIGH);

        H_ACC = Math.toRadians(4);
        driveTo(37, 62, Math.toRadians(335));
        H_ACC = Math.toRadians(1);

        commands.outtake(intake, lift);
    }

    private void scoreInHub(double yPos) {
        lift.setTargetHeight(Levels.HIGH);

        H_ACC = Math.toRadians(4);
        driveTo(37, yPos, Math.toRadians(335));
        H_ACC = Math.toRadians(1);

        commands.outtake(intake, lift);
    }
}
