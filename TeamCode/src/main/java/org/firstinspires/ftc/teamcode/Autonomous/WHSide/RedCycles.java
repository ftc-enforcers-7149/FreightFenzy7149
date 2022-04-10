package org.firstinspires.ftc.teamcode.Autonomous.WHSide;

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
import org.firstinspires.ftc.teamcode.Subsystems.Sensors.ColorSensorFreight;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Levels;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Output;

import java.util.concurrent.atomic.AtomicBoolean;

import static org.firstinspires.ftc.teamcode.GlobalData.H_ACC;
import static org.firstinspires.ftc.teamcode.GlobalData.MIN_TURN;
import static org.firstinspires.ftc.teamcode.GlobalData.POS_ACC;
import static org.firstinspires.ftc.teamcode.GlobalData.SLOW_DIST;
import static org.firstinspires.ftc.teamcode.GlobalData.SPEED_MULT;

@Autonomous(name = "RED WH Cycles")
//@Disabled
public class RedCycles extends Auto_V2_5 {

    private static final Trajectory preload = MecanumDrive.trajectoryBuilder(new Pose2d(6.75, -78.25, Math.toRadians(0)), Math.toRadians(-10))
            .splineToSplineHeading(new Pose2d(12, -78, Math.toRadians(25)), Math.toRadians(10))
            .addTemporalMarker(0.67, () -> GlobalData.openSignal = true) //Open latch just as arm lines up
            .addDisplacementMarker(() -> GlobalData.outtakeSignal = true) //Outtake freight right at the end
            .build();

    private static final Trajectory driveToWall = MecanumDrive.trajectoryBuilder(preload.end(), Math.toRadians(-170))
            .splineToSplineHeading(new Pose2d(-15, -84, Math.toRadians(-95)), Math.toRadians(-170))
            .addTemporalMarker(0.5, () -> GlobalData.armUpSignal = true)
            .build();

    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }

    @Override
    public void auto() {
        distCorrect.startRunning();

        //Initial setup
        drive.setPoseEstimate(new Pose2d(6.75, -78.25, Math.toRadians(0)));
        //Levels levels = commands.detectBarcode(tseDetector);
        Levels level = Levels.LOW;
        SLOW_DIST = 20;
        SPEED_MULT = 1;

        //Score pre-loaded
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        lift.setPower(1);
        waitForTime(250);

        switch (level) {
            case HIGH:
            default:
                scoreInHub(-77, ArmController.ScoringPosition.HIGH);
                break;
            case MIDDLE:
                scoreInHub(-74.5, ArmController.ScoringPosition.MIDDLE_AUTO);
                break;
            case LOW:
                scoreInHub(-71, ArmController.ScoringPosition.LOW_AUTO);
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
            if (getRuntime() >= 24)
                break;

            //Intake and don't hit wall
            intake(Math.max(28 - (cycle * 7), 0));

            //Drive out through gap
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            driveOutOfWarehouse();

            //Drive to and score in hub
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            scoreInHub(-74);

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
        SPEED_MULT = 0.8;
        MIN_TURN = 0.5;

        long startTime = System.currentTimeMillis();

        double startY = drive.getPoseEstimate().getY();

        //Drive towards wall and turn to face warehouse
//        driveTo(() -> {
//                    telemetry.addLine("DRIVE TO WALL");
//
//                    return -4.0;
//                },
//                () -> startY + 4,
//                () -> {
//                    if (System.currentTimeMillis() > startTime + 200)
//                        armController.setScorePos(ArmController.ScoringPosition.UP);
//                    return Math.toRadians(95);
//                }
//                );

        //Strafe into wall
//        long driveStartTime = System.currentTimeMillis();
//        drive.setWeightedDrivePower(new Pose2d(0.05, 0.9, 0.05));
//        customWait(() -> {
//            telemetry.addLine("DRIVE TO WALL");
//
//            return System.currentTimeMillis() < driveStartTime + 50;
//        });

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
        //Start intaking
        armController.setScorePos(ArmController.ScoringPosition.IN);
        intake.setIntakePower(-0.5);

        POS_ACC = 2;
        H_ACC = Math.toRadians(20);

        //Drive through gap
        drive.setWeightedDrivePower(new Pose2d(0.6, -0.35, -0.05));
        long driveStartTime = System.currentTimeMillis();
        customWait(() -> {
            telemetry.addLine("DRIVE INTO WAREHOUSE");

            return System.currentTimeMillis() < driveStartTime + 500;
        });

        intake.setIntakePower(1);

        //Distance correction
        drive.setPoseEstimate(new Pose2d(
                8.5,
                Math.max(Math.min(distCorrect.getFrontDistance(), 100), 10)-144,
                drive.getPoseEstimate().getHeading()));

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
        drive.setWeightedDrivePower(new Pose2d(0.7, -0.2, 0));
        customWait(() -> {
            telemetry.addLine("INTAKE");

            if (distCorrect.getFrontDistance() < 60) {
                if (drive.getPoseEstimate().getHeading() < Math.toRadians(-95)) {
                    drive.setWeightedDrivePower(new Pose2d(
                            Math.min(Math.pow(distCorrect.getFrontDistance()-5-distanceFromWall, 2) * 0.00025 + 0.125, 0.7), //Slow down as approaches wall
                            0.15, //Drive away from wall
                            0.2));
                }
                else if (drive.getPoseEstimate().getHeading() > Math.toRadians(-85)) {
                    drive.setWeightedDrivePower(new Pose2d(
                            Math.min(Math.pow(distCorrect.getFrontDistance()-5-distanceFromWall, 2) * 0.00025 + 0.125, 0.7), //Slow down as approaches wall
                            0.15, //Drive away from wall
                            -0.2));
                }
                else {
                    drive.setWeightedDrivePower(new Pose2d(
                            Math.min(Math.pow(distCorrect.getFrontDistance()-5-distanceFromWall, 2) * 0.00025 + 0.125, 0.7), //Slow down as approaches wall
                            0.15, //Drive away from wall
                            0));
                }
            }

            //Distance correction
            if (distCorrect.getFrontDistance() < 40) {
                drive.setPoseEstimate(new Pose2d(
                        10,
                        distCorrect.getFrontDistance() - 144,
                        drive.getPoseEstimate().getHeading()));

                distCorrect.sensor.disableMoving();
            }

            if (distCorrect.getFrontDistance() < 50 && intake.getFreightInIntake()) return false;
            if (System.currentTimeMillis() > driveStartTime + 1000) return false;

            if (System.currentTimeMillis() > driveStartTime + 500)
                return distCorrect.getFrontDistance() > distanceFromWall;

            return true;
        });
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));

        distCorrect.sensor.enableMoving();
        intake.stopScanningIntake();
    }

    private void driveOutOfWarehouse() {
        POS_ACC = 3;
        SLOW_DIST = 2;
        SPEED_MULT = 1;

        //Make sure robot is against wall
        //kyle loves u.
        drive.setDrivePower(new Pose2d(-0.5, 0, 0.5));
        customWait(() -> distCorrect.getFrontDistance() < 50 &&
                drive.getPoseEstimate().getHeading() < Math.toRadians(-94));
        drive.setWeightedDrivePower(new Pose2d(-0.3, -0.7, 0));
        waitForTime(150);

        //Distance correction
        if (distCorrect.getFrontDistance() < 50) {
            drive.setPoseEstimate(new Pose2d(
                    8.5,
                    distCorrect.getFrontDistance() - 144,
                    drive.getPoseEstimate().getHeading()));
        }

        drive.setWeightedDrivePower(new Pose2d(-0.65, -0.35, 0));

        AtomicBoolean throughGap = new AtomicBoolean(false);
        AtomicBoolean spitOut = new AtomicBoolean(false);

        long driveStartTime = System.currentTimeMillis();

        customWait(() -> {
            telemetry.addLine("DRIVE OUT OF WAREHOUSE");

            if (distCorrect.getFrontDistance() > 40 && !throughGap.get()) {
                double robotHeading = drive.getPoseEstimate().getHeading();
                drive.setPoseEstimate(new Pose2d(7, -84, robotHeading));

                armController.setScorePos(ArmController.ScoringPosition.UP);

                throughGap.set(true);
            }

            //Distance correction
            if (distCorrect.getFrontDistance() < 70 && distCorrect.getFrontDistance() > 15) {
                drive.setPoseEstimate(new Pose2d(
                        6.5,
                        distCorrect.getFrontDistance() - 144,
                        drive.getPoseEstimate().getHeading()));
            }

            if (distCorrect.getFrontDistance() > 35 && !spitOut.get()) {
                intake.spitOutTwo();
                spitOut.set(true);
            }

            return System.currentTimeMillis() < driveStartTime + 1500 &&
                    (System.currentTimeMillis() < driveStartTime + 750 || distCorrect.getFrontDistance() < 45);
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
        //Put arm up
        lift.setPower(1);

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
                    if (System.currentTimeMillis() < startTime + 50)
                        return Math.toRadians(-10);
                    else {
                        if (intake.getFreightType() == ColorSensorFreight.Freight.BALL)
                            armController.setScorePos(ArmController.ScoringPosition.MIDDLE_AUTO);
                        else
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
        waitForTime(150);
    }

    private void scoreInHub(double yPos, ArmController.ScoringPosition scorePos) {
        armController.setScorePos(scorePos);
        waitForTime(100);

        SPEED_MULT = 0.8;
        SLOW_DIST = 20;
        H_ACC = Math.toRadians(4);

        long startTime = System.currentTimeMillis();

        //Drive to hub
        driveTo(() -> 12.0,
                () -> yPos,
                () -> {
                    //Wait for arm to get up before turning to hub
                    if (System.currentTimeMillis() < startTime + 50)
                        return Math.toRadians(0);
                    else {
                        return Math.toRadians(25);
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
