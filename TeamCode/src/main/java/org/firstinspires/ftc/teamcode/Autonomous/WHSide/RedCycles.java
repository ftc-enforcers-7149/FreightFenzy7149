package org.firstinspires.ftc.teamcode.Autonomous.WHSide;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.Auto_V2_5;
import org.firstinspires.ftc.teamcode.GlobalData;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.ArmController;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.MotorIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Levels;

import java.util.concurrent.atomic.AtomicBoolean;

import static org.firstinspires.ftc.teamcode.GlobalData.H_ACC;
import static org.firstinspires.ftc.teamcode.GlobalData.MIN_TURN;
import static org.firstinspires.ftc.teamcode.GlobalData.POS_ACC;
import static org.firstinspires.ftc.teamcode.GlobalData.SLOW_DIST;
import static org.firstinspires.ftc.teamcode.GlobalData.SPEED_MULT;

@Autonomous(name = "RED WH Cycles")
//@Disabled
public class RedCycles extends Auto_V2_5 {

    @Override
    protected Alliance getAlliance() {
        return Alliance.RED;
    }

    @Override
    public void auto() {
        distCorrect.startRunning();

        //Initial setup
        drive.setPoseEstimate(new Pose2d(6.75, -78.25, Math.toRadians(0)));
        //Levels levels = commands.detectBarcode(tseDetector);
        Levels levels = Levels.HIGH;
        SLOW_DIST = 20;
        SPEED_MULT = 1;

        //Score pre-loaded
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        lift.setPower(1); //Bring arm out of robot
        waitForTime(100);

        switch (levels) {
            case HIGH:
            default:
                scoreInHub(-77, ArmController.ScoringPosition.HIGH);
                break;
            case MIDDLE:
                scoreInHub(-75, ArmController.ScoringPosition.MIDDLE_AUTO);
                break;
            case LOW:
                scoreInHub(-73, ArmController.ScoringPosition.LOW_AUTO);
                break;
        }

        int cycle = 0;

        while (opModeIsActive() && cycle < 4) {
            //Align with warehouse
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            driveToWall();

            //Drive through gap
            driveIntoWarehouse();

            //Park if running out of time
            if (getRuntime() >= 29)
                break;

            //Intake and don't hit wall
            intake(Math.max(17 - (cycle * 5), 0));

            //Drive out through gap
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            driveOutOfWarehouse();

            //Drive to and score in hub
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            scoreInHub(-81);

            cycle++;
        }

        intake.setLatch(MotorIntake.LatchPosition.OPEN);
        intake.setPaddle(MotorIntake.PaddlePosition.BACK);

        //Park
        driveToWall();
        driveIntoWarehouse();

        armController.setScorePos(ArmController.ScoringPosition.IN);
        intake.setLatch(MotorIntake.LatchPosition.CLOSED);
        intake.setIntakePower(-0.3);
        SPEED_MULT = 1.0;
        POS_ACC = 3;
        SLOW_DIST = 5;
        driveTo(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY() - 20, Math.toRadians(-90));
        driveTo(drive.getPoseEstimate().getX() + 2, drive.getPoseEstimate().getY() - 6, 0);
        SLOW_DIST = 20;
        POS_ACC = 1;
        intake.setIntakePower(0);
    }

    private void driveToWall() {
        H_ACC = Math.toRadians(5);
        POS_ACC = 2;
        SLOW_DIST = 10;
        SPEED_MULT = 0.9;
        MIN_TURN = 0.6;

        long startTime = System.currentTimeMillis();

        double startY = drive.getPoseEstimate().getY();

        //Drive towards wall and turn to face warehouse
        driveTo(() -> {
                    telemetry.addLine("DRIVE TO WALL");

                    return -5.0;
                },
                () -> startY - 4,
                () -> {
                    if (System.currentTimeMillis() > startTime + 200)
                        armController.setScorePos(ArmController.ScoringPosition.UP);
                    return Math.toRadians(-93);
                }
                );

        //Strafe into wall
        long driveStartTime = System.currentTimeMillis();
        drive.setWeightedDrivePower(new Pose2d(0.05, -0.9, -0.05));
        customWait(() -> {
            telemetry.addLine("DRIVE TO WALL");

            return System.currentTimeMillis() < driveStartTime + 50;
        });

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
        //Start intaking
        armController.setScorePos(ArmController.ScoringPosition.IN);
        intake.setIntakePower(-0.5);

        POS_ACC = 2;
        H_ACC = Math.toRadians(20);

        //Drive through gap
        drive.setWeightedDrivePower(new Pose2d(0.87, -0.1, -0.03));
        long driveStartTime = System.currentTimeMillis();
        customWait(() -> {
            telemetry.addLine("DRIVE INTO WAREHOUSE");

            return Math.abs(deltaHeading(drive.getPoseEstimate().getHeading(), Math.toRadians(-90))) > 5 ||
                    (System.currentTimeMillis() < driveStartTime + 750 &&
                    distCorrect.getFrontDistance() > 45);
        });

        intake.setIntakePower(1);

        //Distance correction
        drive.setPoseEstimate(new Pose2d(
                8.5,
                Math.max(Math.min(distCorrect.getFrontDistance(), 100), 10)-144,
                Math.toRadians(-90)));

        POS_ACC = 1;
        H_ACC = Math.toRadians(1);
    }

    private void intake(double distanceFromWall) {
        //Start intaking
        armController.setScorePos(ArmController.ScoringPosition.IN);
        intake.setIntakePower(1);

        //Drive towards back, intaking
        long driveStartTime = System.currentTimeMillis();
        drive.setWeightedDrivePower(new Pose2d(0.8, -0.2, 0));
        customWait(() -> {
            telemetry.addLine("INTAKE");

            if (distCorrect.getFrontDistance() < 50) {
                if (drive.getPoseEstimate().getHeading() < Math.toRadians(-95)) {
                    drive.setWeightedDrivePower(new Pose2d(
                            Math.min(Math.pow(distCorrect.getFrontDistance()-10-distanceFromWall, 2) * 0.0005 + 0.03, 0.8), //Slow down as approaches wall
                            0.1, //Drive away from wall
                            0.2));
                }
                else if (drive.getPoseEstimate().getHeading() > Math.toRadians(-85)) {
                    drive.setWeightedDrivePower(new Pose2d(
                            Math.min(Math.pow(distCorrect.getFrontDistance()-10-distanceFromWall, 2) * 0.0005 + 0.03, 0.8), //Slow down as approaches wall
                            0.1, //Drive away from wall
                            -0.2));
                }
                else {
                    drive.setWeightedDrivePower(new Pose2d(
                            Math.min(Math.pow(distCorrect.getFrontDistance()-10-distanceFromWall, 2) * 0.0005 + 0.03, 0.8), //Slow down as approaches wall
                            0.1, //Drive away from wall
                            0));
                }
            }

            //Distance correction
            if (distCorrect.getFrontDistance() < 35) {
                drive.setPoseEstimate(new Pose2d(
                        10,
                        distCorrect.getFrontDistance() - 144,
                        drive.getPoseEstimate().getHeading()));

                distCorrect.sensor.disableMoving();
            }

            if (intake.getFreightInIntake()) return false;
            if (System.currentTimeMillis() > driveStartTime + 1000) return false;

            if (System.currentTimeMillis() > driveStartTime + 500)
                return !(distCorrect.getFrontDistance() <= 40) ||
                        !(distCorrect.getFrontDistance() <= distanceFromWall);

            return true;
        });
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));

        distCorrect.sensor.enableMoving();
    }

    private void driveOutOfWarehouse() {
        POS_ACC = 3;
        SLOW_DIST = 2;
        SPEED_MULT = 1;

        //Make sure robot is against wall
        //kyle loves u.
        drive.setDrivePower(new Pose2d(-0.5, 0, 0.5));
        customWait(() -> drive.getPoseEstimate().getHeading() < Math.toRadians(-90));
        drive.setWeightedDrivePower(new Pose2d(-0.7, -0.3, 0));
        waitForTime(150);

        //Distance correction
        if (distCorrect.getFrontDistance() < 40) {
            drive.setPoseEstimate(new Pose2d(
                    8.5,
                    distCorrect.getFrontDistance() - 144,
                    drive.getPoseEstimate().getHeading()));
        }

        drive.setWeightedDrivePower(new Pose2d(-0.7, -0.3, 0));

        AtomicBoolean throughGap = new AtomicBoolean(false);
        AtomicBoolean spitOut = new AtomicBoolean(false);

        long driveStartTime = System.currentTimeMillis();

        customWait(() -> {
            telemetry.addLine("DRIVE OUT OF WAREHOUSE");

            if (distCorrect.getFrontDistance() > 40 && !throughGap.get()) {
                double robotHeading = drive.getPoseEstimate().getHeading();
                drive.setPoseEstimate(new Pose2d(7, -80, robotHeading));

                armController.setScorePos(ArmController.ScoringPosition.UP);

                throughGap.set(true);
            }

            //Distance correction
            if (distCorrect.getFrontDistance() < 70 && distCorrect.getFrontDistance() > 15) {
                drive.setPoseEstimate(new Pose2d(
                        6.5,
                        distCorrect.getFrontDistance() - 144 + 5,
                        drive.getPoseEstimate().getHeading()));
            }

            if (distCorrect.getFrontDistance() > 35 && !spitOut.get()) {
                intake.spitOutTwo();
                spitOut.set(true);
            }

            return System.currentTimeMillis() < driveStartTime + 1000 || distCorrect.getFrontDistance() < 50;
        });

        //Distance correction
        if (distCorrect.getFrontDistance() < 100 && distCorrect.getFrontDistance() > 15) {
            drive.setPoseEstimate(new Pose2d(
                    6.5,
                    distCorrect.getFrontDistance() - 144,
                    drive.getPoseEstimate().getHeading()));
        }

        POS_ACC = 1;
        SPEED_MULT = 1;
        SLOW_DIST = 20;

        //Stop spit out two
        intake.cancelSpitOut();
        intake.setLatch(MotorIntake.LatchPosition.CLOSED);
        intake.setIntakePower(0);
    }

    private void scoreInHub(double yPos) {
        //Put arm out
        armController.setScorePos(ArmController.ScoringPosition.UP);

        SPEED_MULT = 1;
        SLOW_DIST = 20;
        H_ACC = Math.toRadians(4);
        MIN_TURN = 0.25;

        long startTime = System.currentTimeMillis();

        //Drive to hub
        driveTo(() -> 11.5,
                () -> yPos,
                () -> {
                    //Wait for arm to get up before turning to hub
                    if (System.currentTimeMillis() < startTime + 150)
                        return Math.toRadians(-10);
                    else {
                        armController.setScorePos(ArmController.ScoringPosition.HIGH);
                        return Math.toRadians(10);
                    }
                },
                1500);

        H_ACC = Math.toRadians(1);
        SLOW_DIST = 20;
        SPEED_MULT = 1;
        MIN_TURN = 0.15;

        commands.outtake(intake);
    }

    private void scoreInHub(double yPos, ArmController.ScoringPosition scorePos) {
        lift.setPower(1);

        SPEED_MULT = 0.9;
        SLOW_DIST = 20;
        H_ACC = Math.toRadians(4);

        long startTime = System.currentTimeMillis();

        //Drive to hub
        driveTo(() -> 12.0,
                () -> yPos,
                () -> {
                    //Wait for arm to get up before turning to hub
                    if (System.currentTimeMillis() < startTime + 150)
                        return Math.toRadians(0);
                    else {
                        armController.setScorePos(scorePos);
                        return Math.toRadians(25);
                    }
                },
                1500);

        H_ACC = Math.toRadians(1);
        SLOW_DIST = 20;
        SPEED_MULT = 1;

        commands.outtake(intake);
    }
}
