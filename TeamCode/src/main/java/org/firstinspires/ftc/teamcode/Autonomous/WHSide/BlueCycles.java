package org.firstinspires.ftc.teamcode.Autonomous.WHSide;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.Auto_V2_5;
import org.firstinspires.ftc.teamcode.GlobalData;
import org.firstinspires.ftc.teamcode.Odometry.DriveWheels.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.ArmController;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.MotorIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Input;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Levels;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Output;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

import static org.firstinspires.ftc.teamcode.GlobalData.H_ACC;
import static org.firstinspires.ftc.teamcode.GlobalData.MIN_TURN;
import static org.firstinspires.ftc.teamcode.GlobalData.POS_ACC;
import static org.firstinspires.ftc.teamcode.GlobalData.SLOW_DIST;
import static org.firstinspires.ftc.teamcode.GlobalData.SPEED_MULT;

@Autonomous(name = "Blue WH Cycles")
//@Disabled
public class BlueCycles extends Auto_V2_5 {

    private static final Trajectory preload = MecanumDrive.trajectoryBuilder(new Pose2d(6.75, 78.25, Math.toRadians(0)), Math.toRadians(10))
            .splineToSplineHeading(new Pose2d(12, 78, Math.toRadians(-25)), Math.toRadians(-10))
            .addTemporalMarker(0.67, () -> GlobalData.openSignal = true) //Open latch just as arm lines up
            .addDisplacementMarker(() -> GlobalData.outtakeSignal = true) //Outtake freight right at the end
            .build();

    private static final Trajectory driveToWall = MecanumDrive.trajectoryBuilder(preload.end(), Math.toRadians(170))
            .splineToSplineHeading(new Pose2d(-25, 80, Math.toRadians(100)), Math.toRadians(160))
            .addTemporalMarker(0.5, () -> GlobalData.armUpSignal = true)
            .build();

    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }

    @Override
    public void auto() {
        //distCorrect.startRunning();
        distCorrect.sensor.disableMoving();

        //Debug distance and intake sensors with leds
        //RED: no freight
        //GREEN: freight
        addOutput(new Output() {
            @Override
            public void updateOutput() {
                if (intake.getFreightInIntake())
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                else
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                }
        });

        //Initial setup
        drive.setPoseEstimate(new Pose2d(6.75, 78.25, Math.toRadians(0)));
        Levels level = commands.detectBarcode(tseDetector);
        SLOW_DIST = 20;
        SPEED_MULT = 1;

        //Score pre-loaded
        lift.setPower(1);
        waitForTime(250);

        switch (level) {
            case HIGH:
            default:
                scoreInHub(77, ArmController.ScoringPosition.HIGH);
                break;
            case MIDDLE:
                scoreInHub(74.5, ArmController.ScoringPosition.MIDDLE_AUTO);
                break;
            case LOW:
                scoreInHub(71, ArmController.ScoringPosition.LOW_AUTO);
                break;
        }

        for (int cycle = 0; cycle < 4; cycle++) {
            //Align with warehouse
            driveToWall();

            //Drive through gap
            driveIntoWarehouse();

            //Intake and don't hit wall
            intake(Math.max(28 - (cycle * 5), 5));

            //Drive out through gap
            driveOutOfWarehouse();

            //Drive to and score in hub
            scoreInHub(81);

            if (getRuntime() > 23.5) break;
        }

        driveToWall();
        //driveIntoWarehouse();

        //Park

        armController.setScorePos(ArmController.ScoringPosition.IN);
        intake.setLatch(MotorIntake.LatchPosition.CLOSED);
        intake.setIntakePower(-0.3);
        SPEED_MULT = 1;
        POS_ACC = 3;
        SLOW_DIST = 5;

        drive.setWeightedDrivePower(new Pose2d(1, 0, 0));
        waitForTime(900);

        //driveTo(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY() + 20, Math.toRadians(90));
        //driveTo(drive.getPoseEstimate().getX() + 2, drive.getPoseEstimate().getY() + 6, 0);
        SLOW_DIST = 20;
        POS_ACC = 1;
        intake.setIntakePower(0);

        distCorrect.sensor.setHighPass(false);
    }

    private void driveToWall() {
        H_ACC = Math.toRadians(5);
        POS_ACC = 2;
        SLOW_DIST = 10;
        SPEED_MULT = 0.8;
        MIN_TURN = 0.5;

        double startY = drive.getPoseEstimate().getY();

        drive.followTrajectoryAsync(driveToWall);
        waitForDriveComplete();

        //Bring arm to ground
        armController.setScorePos(ArmController.ScoringPosition.IN);
        intake.setIntakePower(-0.5);

        //Distance correction
        drive.setPoseEstimate(new Pose2d(
                6.5,
                startY,
                drive.getPoseEstimate().getHeading()));

        //Reset defaults
        H_ACC = Math.toRadians(1);
        POS_ACC = 1;
        SLOW_DIST = 20;
        SPEED_MULT = 1;
        MIN_TURN = 0.15;
    }

    private void driveIntoWarehouse() {
        distCorrect.startRunning();

        //Start intaking
        armController.setScorePos(ArmController.ScoringPosition.IN);
        intake.setIntakePower(-0.5);

        POS_ACC = 2;
        H_ACC = Math.toRadians(20);

        //Drive through gap
        drive.setWeightedDrivePower(new Pose2d(1, 0, 0));
        waitForTime(600);
        drive.setWeightedDrivePower(new Pose2d());

        intake.setIntakePower(1);

        //Distance correction
        long sensorStartTime = System.currentTimeMillis();
        AtomicReference<Double> total = new AtomicReference<Double>(0d);
        AtomicInteger loops = new AtomicInteger(0);
        customWait(() -> {
            total.set(total.get() + distCorrect.getFrontDistance());
            loops.set(loops.get() + 1);

            Log.d("Average distance: ", String.valueOf(total.get()/loops.get()));

            //return distCorrect.getFrontDistance() > 100 &&
            return        System.currentTimeMillis() <= sensorStartTime + 80;
        });
        if (total.get()/loops.get() < 60) {
            drive.setPoseEstimate(new Pose2d(
                    8.5,
                    144 - total.get()/loops.get(),
                    drive.getPoseEstimate().getHeading()));
        }

        POS_ACC = 1;
        H_ACC = Math.toRadians(1);
    }

    private void intake(double distanceFromWall) {
        intake.startScanningIntake();

        //Start intaking
        armController.setScorePos(ArmController.ScoringPosition.IN);
        intake.setIntakePower(1);

        //Drive towards back, intaking
        long driveStartTime = System.currentTimeMillis();
        drive.setWeightedDrivePower(new Pose2d(0.7, 0.2, 0));
        customWait(() -> {
            telemetry.addLine("INTAKE");

            if (deltaHeading(drive.getPoseEstimate().getHeading(), Math.toRadians(95)) < 0)  {
                drive.setWeightedDrivePower(new Pose2d(
                        Math.max(Math.min(
                                Math.pow(Math.max(144-drive.getPoseEstimate().getY()-distanceFromWall, 0), 2) * 0.00022 + 0.15,
                                0.7), 0.1), //Slow down as approaches wall
                        -Math.min(Math.pow(144-drive.getPoseEstimate().getY(), 2) * 0.00006 + 0.05, 0.2), //Drive away from wall
                        -0.1));
            }
            else if (deltaHeading(drive.getPoseEstimate().getHeading(), Math.toRadians(85)) > 0) {
                drive.setWeightedDrivePower(new Pose2d(
                        Math.max(Math.min(
                                Math.pow(Math.max(144-drive.getPoseEstimate().getY()-distanceFromWall, 0), 2) * 0.00022 + 0.15,
                                0.7), 0.1), //Slow down as approaches wall
                        -Math.min(Math.pow(144-drive.getPoseEstimate().getY(), 2) * 0.00006 + 0.05, 0.2), //Drive away from wall
                        0.1));
            }
            else {
                drive.setWeightedDrivePower(new Pose2d(
                        Math.max(Math.min(
                                Math.pow(Math.max(144-drive.getPoseEstimate().getY()-distanceFromWall, 0), 2) * 0.00022 + 0.15,
                                0.7), 0.1), //Slow down as approaches wall
                        -Math.min(Math.pow(144-drive.getPoseEstimate().getY(), 2) * 0.00006 + 0.05, 0.2), //Drive away from wall
                        0));
            }

            //Distance correction
            if (distCorrect.getFrontDistance() < 50 && distCorrect.getFrontDistance() > 22) {
                drive.setPoseEstimate(new Pose2d(
                        10,
                        144 - distCorrect.getFrontDistance(),
                        drive.getPoseEstimate().getHeading()));
            }

            if (distCorrect.getFrontDistance() < 43 && intake.getFreightInIntake()) return false;
            if (System.currentTimeMillis() > driveStartTime + 1000) return false;

            return true;
        });
        drive.setWeightedDrivePower(new Pose2d());

        armController.setScorePos(ArmController.ScoringPosition.PARTIAL_UP);
        intake.setLatch(MotorIntake.LatchPosition.CLOSED);

        intake.stopScanningIntake();
    }

    private void driveOutOfWarehouse() {
        POS_ACC = 3;
        SLOW_DIST = 2;
        SPEED_MULT = 1;

        armController.setScorePos(ArmController.ScoringPosition.PARTIAL_UP);
        intake.setIntakePower(1);

        //Make sure robot is against wall
        //kyle loves u.
        drive.setWeightedDrivePower(new Pose2d(-0.5, 0.1, -0.4));
        customWait(() -> deltaHeading(drive.getPoseEstimate().getHeading(), Math.toRadians(93)) < 0);
        intake.setIntakePower(-1);
        drive.setWeightedDrivePower(new Pose2d(-0.4, 0.6, 0));
        waitForTime(200);

        armController.setScorePos(ArmController.ScoringPosition.UP);

        drive.setWeightedDrivePower(new Pose2d(-0.55, 0.45, 0));

        long driveStartTime = System.currentTimeMillis();

        customWait(() -> {
            telemetry.addLine("DRIVE OUT OF WAREHOUSE");

            //Distance correction
            if (distCorrect.getFrontDistance() < 50 && distCorrect.getFrontDistance() > 22) {
                drive.setPoseEstimate(new Pose2d(
                        10,
                        144 - distCorrect.getFrontDistance() - 11.5,
                        drive.getPoseEstimate().getHeading()));
            }

            if (drive.getPoseEstimate().getY() < 110) {
                drive.setWeightedDrivePower(new Pose2d(-0.75, 0.25, 0));
                intake.setIntakePower(0);
            }

            return System.currentTimeMillis() < driveStartTime + 1000 &&
                    drive.getPoseEstimate().getY() > 85;
        });
        drive.setWeightedDrivePower(new Pose2d());

        drive.setPoseEstimate(new Pose2d(6.5, drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading()));

        POS_ACC = 1;
        SPEED_MULT = 1;
        SLOW_DIST = 20;

        intake.setIntakePower(0);
        distCorrect.stopRunning();
    }

    private void scoreInHub(double yPos) {
        //Put arm up
        armController.setScorePos(ArmController.ScoringPosition.UP);

        SPEED_MULT = 1;
        SLOW_DIST = 20;
        H_ACC = Math.toRadians(4);
        MIN_TURN = 0.25;

        long startTime = System.currentTimeMillis();

        AtomicBoolean armOut = new AtomicBoolean(false);

        //Drive to hub
        driveTo(() -> 13.5,
                () -> yPos,
                () -> {
                    //Wait for arm to get up before turning to hub
                    if (System.currentTimeMillis() < startTime + 50)
                        return Math.toRadians(5);
                    else if (!armOut.get()) {
                        armController.setScorePos(ArmController.ScoringPosition.HIGH_AUTO);

                        armOut.set(true);

                        return Math.toRadians(-17);
                    }
                    else
                        return Math.toRadians(-17);
                },
                1500);

        H_ACC = Math.toRadians(1);
        SLOW_DIST = 20;
        SPEED_MULT = 1;
        MIN_TURN = 0.15;

        commands.outtake(intake);
        waitForTime(150);
    }

    private void scoreInHub(double yPos, ArmController.ScoringPosition scorePos) {
        armController.setScorePos(scorePos);
        waitForTime(100);

        SPEED_MULT = 1;
        SLOW_DIST = 15;
        H_ACC = Math.toRadians(4);

        long startTime = System.currentTimeMillis();

        //Drive to hub
        driveTo(() -> 12.5,
                () -> yPos,
                () -> {
                    //Wait for arm to get up before turning to hub
                    if (System.currentTimeMillis() < startTime + 50)
                        return Math.toRadians(0);
                    else {
                        if (scorePos == ArmController.ScoringPosition.HIGH_AUTO)
                            return Math.toRadians(-28);
                        else
                            return Math.toRadians(-25);
                    }
                },
                1500);

        H_ACC = Math.toRadians(1);
        SLOW_DIST = 20;
        SPEED_MULT = 1;

        commands.outtake(intake);
        waitForTime(150);
    }
}
