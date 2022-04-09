package org.firstinspires.ftc.teamcode.Autonomous.WHSide;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.Auto_V2_5;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.ArmController;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.MotorIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Levels;

import java.util.concurrent.atomic.AtomicBoolean;

import static org.firstinspires.ftc.teamcode.GlobalData.H_ACC;
import static org.firstinspires.ftc.teamcode.GlobalData.MIN_TURN;
import static org.firstinspires.ftc.teamcode.GlobalData.POS_ACC;
import static org.firstinspires.ftc.teamcode.GlobalData.SLOW_DIST;
import static org.firstinspires.ftc.teamcode.GlobalData.SPEED_MULT;

@Autonomous(name = "Blue WH Cycles NEW")
//@Disabled
public class BlueCyclesNEW extends Auto_V2_5 {

    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }

    @Override
    public void auto() {
        distCorrect.startRunning();

        //Initial setup
        drive.setPoseEstimate(new Pose2d(6.75, 78.25, 0));
        //Levels levels = commands.detectBarcode(tseDetector);
        Levels levels = Levels.HIGH;
        SLOW_DIST = 20;
        SPEED_MULT = 1;

        //Score pre-loaded
        /*H_ACC = Math.toRadians(4);

        armController.setScorePos(ArmController.ScoringPosition.UP);
        waitForTime(150);

        switch (levels) { //Go to scoring position for corresponding level
            default:
            case HIGH:
                armController.setScorePos(ArmController.ScoringPosition.HIGH);
                driveTo(12, 71, Math.toRadians(337.5));
                break;
            case MIDDLE:
                armController.setScorePos(ArmController.ScoringPosition.MIDDLE);
                driveTo(12, 71, Math.toRadians(337.5));
                break;
            case LOW:
                armController.setScorePos(ArmController.ScoringPosition.LOW);
                driveTo(12, 71, Math.toRadians(337.5));
                break;
        }

        H_ACC = Math.toRadians(1);

        commands.outtake(intake, lift);*/

        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        armController.setScorePos(ArmController.ScoringPosition.UP);
        waitForTime(100);

        switch (levels) {
            case HIGH:
            default:
                scoreInHub(80, ArmController.ScoringPosition.HIGH);
                break;
            case MIDDLE:
                scoreInHub(80, ArmController.ScoringPosition.MIDDLE);
                break;
            case LOW:
                scoreInHub(80, ArmController.ScoringPosition.LOW);
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
            intake(Math.max(25 - (cycle * 5), 5));

            //Drive out through gap
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            driveOutOfWarehouse();

            //Drive to and score in hub
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            scoreInHub(77);

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
        SLOW_DIST = 5;
        driveTo(drive.getPoseEstimate().getX() + 2, drive.getPoseEstimate().getY() + 6, 0);
        SLOW_DIST = 20;
        intake.setIntakePower(0);
    }

    private void driveToWall() {
        //distCorrect.startSideSensor();

        H_ACC = Math.toRadians(5);
        POS_ACC = 2;
        SLOW_DIST = 10;
        SPEED_MULT = 0.9;
        MIN_TURN = 0.4;

        long startTime = System.currentTimeMillis();

        double startY = drive.getPoseEstimate().getY();

        //Drive towards wall and turn to face warehouse
        driveTo(() -> {
                    telemetry.addLine("DRIVE TO WALL");

                    //if (Math.abs(deltaHeading(drive.getPoseEstimate().getHeading(), Math.toRadians(80))) > Math.toRadians(3))
                    //    return 13.0;
                    //else {
                    //    return -10.0;
                    //}

                    return -4.0;
                },
                () -> startY + 4,
                () -> {
                    if (System.currentTimeMillis() > startTime + 200)
                        armController.setScorePos(ArmController.ScoringPosition.UP);
                    return Math.toRadians(95);
                }
                );

        //Strafe into wall
        long driveStartTime = System.currentTimeMillis();
        drive.setWeightedDrivePower(new Pose2d(0.05, 0.9, 0.05));
        customWait(() -> {
            telemetry.addLine("DRIVE TO WALL");

            return System.currentTimeMillis() < driveStartTime + 50;
        });

        //Bring arm to ground
        armController.setScorePos(ArmController.ScoringPosition.IN);

        //Distance correction
        drive.setPoseEstimate(new Pose2d(
                6.5,
                startY,
                Math.toRadians(90)));

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
        intake.setIntakePower(1);

        POS_ACC = 2;
        H_ACC = Math.toRadians(20);

        //Drive through gap
        drive.setWeightedDrivePower(new Pose2d(0.65, 0.3, 0.05));
        long driveStartTime = System.currentTimeMillis();
        customWait(() -> {
            telemetry.addLine("DRIVE INTO WAREHOUSE");

            return System.currentTimeMillis() < driveStartTime + 750 &&
                    distCorrect.getFrontDistance() > 35;
        });

        //Distance correction
        drive.setPoseEstimate(new Pose2d(
                8.5,
                144-Math.max(Math.min(distCorrect.getFrontDistance(), 100), 10),
                Math.toRadians(90)));

        POS_ACC = 1;
        H_ACC = Math.toRadians(1);
    }

    private void intake(double distanceFromWall) {
        //distCorrect.sensorF.disableMoving();

        //Start intaking
        armController.setScorePos(ArmController.ScoringPosition.IN);
        intake.setIntakePower(1);

        //Drive towards back, intaking
        long driveStartTime = System.currentTimeMillis();
        drive.setWeightedDrivePower(new Pose2d(0.8, 0.2, 0));
        customWait(() -> {
            telemetry.addLine("INTAKE");

            if (distCorrect.getFrontDistance() < 50) {
                if (drive.getPoseEstimate().getHeading() > Math.toRadians(95)) {
                    drive.setWeightedDrivePower(new Pose2d(
                            Math.min(Math.pow(distCorrect.getFrontDistance()-10-distanceFromWall, 2) * 0.0005, 0.8), //Slow down as approaches wall
                            -0.075, //Drive away from wall
                            -0.2));
                }
                else if (drive.getPoseEstimate().getHeading() < Math.toRadians(85)) {
                    drive.setWeightedDrivePower(new Pose2d(
                            Math.min(Math.pow(distCorrect.getFrontDistance()-10-distanceFromWall, 2) * 0.0005, 0.8), //Slow down as approaches wall
                            -0.075, //Drive away from wall
                            0.2));
                }
                else {
                    drive.setWeightedDrivePower(new Pose2d(
                            Math.min(Math.pow(distCorrect.getFrontDistance()-10-distanceFromWall, 2) * 0.0005, 0.8), //Slow down as approaches wall
                            -0.075, //Drive away from wall
                            0));
                }
            }

            //Distance correction
            if (distCorrect.getFrontDistance() < 30) {
                drive.setPoseEstimate(new Pose2d(
                        10,
                        144 - distCorrect.getFrontDistance(),
                        drive.getPoseEstimate().getHeading()));

                distCorrect.sensorF.disableMoving();
            }

            if (intake.getFreightInIntake()) return false;
            if (System.currentTimeMillis() > driveStartTime + 1000) return false;

            if (System.currentTimeMillis() > driveStartTime + 500)
                return !(distCorrect.getFrontDistance() <= 40) ||
                        !(distCorrect.getFrontDistance() <= distanceFromWall);

            return true;
        });
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));

        distCorrect.sensorF.enableMoving();
    }

    private void driveOutOfWarehouse() {
        POS_ACC = 3;
        SLOW_DIST = 2;
        SPEED_MULT = 1;

        //Make sure robot is against wall
        //kyle loves u.
        drive.setDrivePower(new Pose2d(-0.5, 0, -0.5));
        customWait(() -> drive.getPoseEstimate().getHeading() > Math.toRadians(90));
        drive.setWeightedDrivePower(new Pose2d(-0.7, 0.3, 0));
        waitForTime(150);

        //Distance correction
        if (distCorrect.getFrontDistance() < 30) {
            drive.setPoseEstimate(new Pose2d(
                    8.5,
                    144 - distCorrect.getFrontDistance(),
                    drive.getPoseEstimate().getHeading()));
        }

        drive.setWeightedDrivePower(new Pose2d(-0.7, 0.3, 0));

        AtomicBoolean throughGap = new AtomicBoolean(false);
        AtomicBoolean spitOut = new AtomicBoolean(false);

        long driveStartTime = System.currentTimeMillis();

        customWait(() -> {
            telemetry.addLine("DRIVE OUT OF WAREHOUSE");

            if (distCorrect.getFrontDistance() > 40 && !throughGap.get()) {
                double robotHeading = drive.getPoseEstimate().getHeading();
                drive.setPoseEstimate(new Pose2d(7, 80, robotHeading));

                armController.setScorePos(ArmController.ScoringPosition.UP);

                throughGap.set(true);
            }

            //Distance correction
            if (distCorrect.getFrontDistance() < 70 && distCorrect.getFrontDistance() > 15) {
                drive.setPoseEstimate(new Pose2d(
                        6.5,
                        144 - distCorrect.getFrontDistance() - 5,
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
                    144 - distCorrect.getFrontDistance(),
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
                        return Math.toRadians(10);
                    else {
                        armController.setScorePos(ArmController.ScoringPosition.HIGH);
                        return Math.toRadians(-5);
                    }
                },
                1500);

        H_ACC = Math.toRadians(1);
        SLOW_DIST = 20;
        SPEED_MULT = 1;
        MIN_TURN = 0.15;

        commands.outtake(intake, lift);
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
                    if (System.currentTimeMillis() < startTime + 100)
                        return Math.toRadians(360);
                    else {
                        armController.setScorePos(scorePos);
                        return Math.toRadians(340);
                    }
                },
                1500);

        H_ACC = Math.toRadians(1);
        SLOW_DIST = 20;
        SPEED_MULT = 1;

        commands.outtake(intake, lift);
    }
}
