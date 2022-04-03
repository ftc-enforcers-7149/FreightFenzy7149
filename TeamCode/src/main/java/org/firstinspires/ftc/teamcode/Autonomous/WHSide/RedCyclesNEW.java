package org.firstinspires.ftc.teamcode.Autonomous.WHSide;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.Auto_V2_5;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.ArmController;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.MotorIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Levels;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import static org.firstinspires.ftc.teamcode.GlobalData.H_ACC;
import static org.firstinspires.ftc.teamcode.GlobalData.POS_ACC;
import static org.firstinspires.ftc.teamcode.GlobalData.SLOW_DIST;
import static org.firstinspires.ftc.teamcode.GlobalData.SPEED_MULT;

@Autonomous(name = "Red WH Cycles NEW")
//@Disabled
public class RedCyclesNEW extends Auto_V2_5 {

    @Override
    protected Alliance getAlliance() {
        return Alliance.RED;
    }

    @Override
    public void auto() {
        distCorrect.startRunning();

        //Initial setup
        drive.setPoseEstimate(new Pose2d(6.75, -78.25, 0));
        //Levels levels = commands.detectBarcode(tseDetector);
        Levels levels = Levels.HIGH;
        SLOW_DIST = 20;
        SPEED_MULT = 0.92;

        //Score pre-loaded
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

        H_ACC = Math.toRadians(4);

        armController.setScorePos(ArmController.ScoringPosition.UP);
        waitForTime(150);

        switch (levels) { //Go to scoring position for corresponding level
            default:
            case HIGH:
                armController.setScorePos(ArmController.ScoringPosition.HIGH);
                driveTo(12, -71, Math.toRadians(22.5));
                break;
            case MIDDLE:
                armController.setScorePos(ArmController.ScoringPosition.MIDDLE);
                driveTo(12, -71, Math.toRadians(22.5));
                break;
            case LOW:
                armController.setScorePos(ArmController.ScoringPosition.LOW);
                driveTo(12, -71, Math.toRadians(22.5));
                break;
        }

        H_ACC = Math.toRadians(1);

        commands.outtake(intake, lift);

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
            //if (!intake.getFreightInIntake())
                intake(Math.max(15 - (cycle * 6), 5));

            //Drive out through gap
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            driveOutOfWarehouse();

            //Drive to and score in hub
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            scoreInHub(-66);

            cycle++;
        }

        intake.setLatch(MotorIntake.LatchPosition.OPEN);
        intake.setPaddle(MotorIntake.PaddlePosition.BACK);

        //Park
        driveToWall();
        driveIntoWarehouse();
        intake(50);

        armController.setScorePos(ArmController.ScoringPosition.IN);
        intake.setLatch(MotorIntake.LatchPosition.CLOSED);
        intake.setIntakePower(-0.3);
        SPEED_MULT = 1.0;
        SLOW_DIST = 5;
        driveTo(drive.getPoseEstimate().getX() + 2, drive.getPoseEstimate().getY() - 6, 0);
        SLOW_DIST = 20;
        intake.setIntakePower(0);
    }

    private void driveToWall() {
        //distCorrect.startSideSensor();

        H_ACC = Math.toRadians(5);
        POS_ACC = 2;
        SLOW_DIST = 10;
        SPEED_MULT = 0.9;

        //Drive towards wall and turn to face warehouse
        driveTo(() -> {
                    telemetry.addLine("DRIVE TO WALL");

                    if (Math.abs(deltaHeading(drive.getPoseEstimate().getHeading(), Math.toRadians(275))) > Math.toRadians(3))
                        return 13.0;
                    else {
                        return drive.getPoseEstimate().getX();
                    }
                },
                () -> -75.0,
                () -> Math.toRadians(275)
                );

        armController.setScorePos(ArmController.ScoringPosition.UP);

        //Strafe into wall
        long driveStartTime = System.currentTimeMillis();
        drive.setWeightedDrivePower(new Pose2d(0.01, -0.7, 0));
        customWait(() -> {
            telemetry.addLine("DRIVE TO WALL");

            return distCorrect.getSideWall() > 11 && System.currentTimeMillis() < driveStartTime + 1000;
        });
        waitForTime(500);

        //Bring arm to ground
        armController.setScorePos(ArmController.ScoringPosition.IN);

        //Distance correction
        drive.setPoseEstimate(new Pose2d(
                8.5,
                -75,
                Math.toRadians(270)));

        //Reset defaults
        H_ACC = Math.toRadians(1);
        POS_ACC = 1;
        SLOW_DIST = 20;
        SPEED_MULT = 0.92;

        //distCorrect.stopRunning();
    }

    private void driveIntoWarehouse() {
        //distCorrect.startRunning();
        intake.startScanningIntake();

        //Start intaking
        armController.setScorePos(ArmController.ScoringPosition.IN);
        intake.setIntakePower(1);

        POS_ACC = 2;
        H_ACC = Math.toRadians(20);

        //Drive through gap
        drive.setWeightedDrivePower(new Pose2d(0.8, -0.6, 0));
        long driveStartTime = System.currentTimeMillis();
        customWait(() -> {
            telemetry.addLine("DRIVE INTO WAREHOUSE");

            return System.currentTimeMillis() < driveStartTime + 750;
        });

        //Distance correction
        drive.setPoseEstimate(new Pose2d(
                8.5,
                -144+Math.max(Math.min(distCorrect.getFrontDistance(), 100), 10),
                Math.toRadians(270)));

        POS_ACC = 1;
        H_ACC = Math.toRadians(1);

        intake.stopScanningIntake();
        //distCorrect.stopRunning();
    }

    private void intake(double distanceFromWall) {
        //distCorrect.startRunning();
        intake.startScanningIntake();

        //Start intaking
        armController.setScorePos(ArmController.ScoringPosition.IN);
        intake.setIntakePower(1);

        //Drive towards back, intaking
        long driveStartTime = System.currentTimeMillis();
        drive.setWeightedDrivePower(new Pose2d(0.7, -0.5, 0));
        customWait(() -> {
            telemetry.addLine("INTAKE");

            if (distCorrect.getFrontDistance() < 50)
                drive.setWeightedDrivePower(new Pose2d(
                        Math.min(Math.pow(distCorrect.getFrontDistance(), 2) * 0.0003, 0.7), //Slow down as approaches wall
                        distanceFromWall * (0.2 / 22), //Drive away from wall
                        0));

            return ((!intake.getFreightInIntake() &&
                    distCorrect.getFrontDistance() > distanceFromWall) ||
                    distCorrect.getFrontDistance() > 30) &&
                    System.currentTimeMillis() < driveStartTime + 2000;
        });
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));

        //Distance correction
        drive.setPoseEstimate(new Pose2d(
                10,
                -144+Math.max(Math.min(distCorrect.getFrontDistance(), 60), 10),
                Math.toRadians(270)));

        intake.stopScanningIntake();
        //distCorrect.stopRunning();
    }

    private void driveOutOfWarehouse() {
        //distCorrect.startRunning();

        POS_ACC = 3;
        SLOW_DIST = 2;
        SPEED_MULT = 0.9;

        //Make sure robot is against wall
        drive.setWeightedDrivePower(new Pose2d(-0.04, -0.6, 0));
        waitForTime(175);

        //Distance correction
        drive.setPoseEstimate(new Pose2d(
                8.5,
                -144+Math.max(Math.min(distCorrect.getFrontDistance(), 60), 10),
                Math.toRadians(270)));

        AtomicReference<Double> lastFrontReading = new AtomicReference<>(distCorrect.getFrontDistance());

        AtomicBoolean throughGap = new AtomicBoolean(false);
        AtomicBoolean spitOut = new AtomicBoolean(false);

        driveTo(() -> {
                    telemetry.addLine("DRIVE OUT OF WAREHOUSE");

                    if (!throughGap.get())
                        return drive.getPoseEstimate().getX() - 15;
                    else
                        return drive.getPoseEstimate().getX();
                },
                () -> {
                    if (distCorrect.getSideWall() > 11 && !throughGap.get())
                        return -75.0;
                    else {
                        SLOW_DIST = 10;

                        return -70.0;
                    }
                },
                () -> {
                    //Distance correction
                    if (distCorrect.getFrontDistance() < 45) {
                        drive.setPoseEstimate(new Pose2d(
                                8.5,
                                -144+Math.max(Math.min(distCorrect.getFrontDistance(), 60), 10),
                                Math.toRadians(270)));

                        lastFrontReading.set(distCorrect.getFrontDistance());
                    }
                    //Through gap, final distance correction
                    else if (!throughGap.get()) {
                        double robotHeading = drive.getPoseEstimate().getHeading();
                        drive.setPoseEstimate(new Pose2d(7, -75, robotHeading));

                        armController.setScorePos(ArmController.ScoringPosition.UP);

                        throughGap.set(true);
                    }

                    //Spit out second block, assuming there is one
                    if (distCorrect.getFrontDistance() > 45 && !spitOut.get()) {
                        intake.spitOutTwo();
                        spitOut.set(true);
                    }

                    if (distCorrect.getSideWall() > 11)
                        return Math.toRadians(271);
                    else
                        return Math.toRadians(270);
                }
                );

        /*drive.setPoseEstimate(new Pose2d(
                8.5,
                drive.getPoseEstimate().getY(),
                Math.toRadians(90)));*/

        POS_ACC = 1;
        SPEED_MULT = 0.92;
        SLOW_DIST = 20;

        //Stop spit out two
        intake.cancelSpitOut();
        intake.setLatch(MotorIntake.LatchPosition.CLOSED);
        intake.setIntakePower(0);

        //distCorrect.stopRunning();
    }

    private void scoreInHub(double yPos) {
        //Put arm out
        //armController.setScorePos(ArmController.ScoringPosition.UP);
        //waitForTime(150);
        armController.setScorePos(ArmController.ScoringPosition.HIGH);
        waitForTime(150);

        SPEED_MULT = 0.9;
        SLOW_DIST = 20;
        H_ACC = Math.toRadians(4);

        //Drive to hub
        driveTo(12, yPos, Math.toRadians(15), 1500);

        H_ACC = Math.toRadians(1);
        SLOW_DIST = 20;
        SPEED_MULT = 0.92;

        commands.outtake(intake, lift);
    }
}
