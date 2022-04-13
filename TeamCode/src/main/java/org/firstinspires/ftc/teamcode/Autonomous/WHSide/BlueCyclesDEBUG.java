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
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Input;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Levels;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Output;

import java.util.concurrent.atomic.AtomicBoolean;

import static org.firstinspires.ftc.teamcode.GlobalData.H_ACC;
import static org.firstinspires.ftc.teamcode.GlobalData.MIN_TURN;
import static org.firstinspires.ftc.teamcode.GlobalData.POS_ACC;
import static org.firstinspires.ftc.teamcode.GlobalData.SLOW_DIST;
import static org.firstinspires.ftc.teamcode.GlobalData.SPEED_MULT;

@Autonomous(name = "BLUE WH Cycles DEBUG")
//@Disabled
public class BlueCyclesDEBUG extends Auto_V2_5 {

    private static final Trajectory preload = MecanumDrive.trajectoryBuilder(new Pose2d(6.75, 78.25, Math.toRadians(0)), Math.toRadians(10))
            .splineToSplineHeading(new Pose2d(12, 78, Math.toRadians(-25)), Math.toRadians(-10))
            .addTemporalMarker(0.67, () -> GlobalData.openSignal = true) //Open latch just as arm lines up
            .addDisplacementMarker(() -> GlobalData.outtakeSignal = true) //Outtake freight right at the end
            .build();

    private static final Trajectory driveToWall = MecanumDrive.trajectoryBuilder(preload.end(), Math.toRadians(170))
            .splineToSplineHeading(new Pose2d(-15, 84, Math.toRadians(95)), Math.toRadians(170))
            .addTemporalMarker(0.5, () -> GlobalData.armUpSignal = true)
            .build();

    private AtomicBoolean sensorState = new AtomicBoolean(true);

    private boolean paused = false;
    private boolean x, lastX;

    private double speedMultiplier = 1.0;
    private boolean dUp, dDown, lastDUp, lastDDown;

    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }

    @Override
    public void auto() {
        distCorrect.startRunning();
        distCorrect.sensor.setHighPass(true, 200);
        distCorrect.sensor.disableMoving();

        //Debug distance and intake sensors with leds
        //RED: no freight
        //YELLOW: freight
        //SOLID: good sensors
        //STROBE: bad sensors
        addOutput(new Output() {
            @Override
            public void updateOutput() {
                if (sensorState.get()) {
                    if (intake.getFreightInIntake())
                        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                    else
                        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                }
                else {
                    if (intake.getFreightInIntake())
                        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
                    else
                        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
                }
            }
        });

        //Use dpad (up and down) to set speed multipliers for all actions
        addInput(new Input() {
            @Override
            public void updateInput() {
                dUp = gamepad1.dpad_up;
                dDown = gamepad1.dpad_down;

                if (dUp && !lastDUp)
                    speedMultiplier = Math.min(speedMultiplier + 0.1, 1);
                else if (dDown && !lastDDown)
                    speedMultiplier = Math.max(speedMultiplier - 0.1, 0);

                lastDUp = dUp;
                lastDDown = dDown;

                telemetry.addData("MULTIPLIER: ", speedMultiplier);
            }
        });

        //Use "x" to pause the opmode between each move
        addInput(new Input() {
            @Override
            public void updateInput() {
                x = gamepad1.x;

                if (x && !lastX)
                    paused = !paused;

                lastX = x;
            }
        });

        //Initial setup
        drive.setPoseEstimate(new Pose2d(6.75, 78.25, Math.toRadians(0)));
        Levels level = commands.detectBarcode(tseDetector);
        //Levels level = Levels.LOW;
        SLOW_DIST = 20;
        SPEED_MULT = speedMultiplier;

        customWait(() -> paused); //Pause before preload

        //Score pre-loaded
        //led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
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

        int cycle = 0;

        while (opModeIsActive() && cycle < 4) {
            //Align with warehouse
            //led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            customWait(() -> paused);
            driveToWall();

            //Drive through gap
            customWait(() -> paused);
            driveIntoWarehouse();

            //Park if running out of time
            if (getRuntime() >= 25)
                break;

            //Intake and don't hit wall
            customWait(() -> paused);
            intake(Math.max(28 - (cycle * 7), 0));

            //Drive out through gap
            //led.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            customWait(() -> paused);
            driveOutOfWarehouse();

            //Drive to and score in hub
            //led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            customWait(() -> paused);
            scoreInHub(84);

            cycle++;
        }

        customWait(() -> paused);

        intake.setLatch(MotorIntake.LatchPosition.OPEN);
        intake.setPaddle(MotorIntake.PaddlePosition.BACK);

        //Park
        customWait(() -> paused);
        driveToWall();

        customWait(() -> paused);
        driveIntoWarehouse();

        customWait(() -> paused);
        armController.setScorePos(ArmController.ScoringPosition.IN);
        intake.setLatch(MotorIntake.LatchPosition.CLOSED);
        intake.setIntakePower(-0.3);
        SPEED_MULT = speedMultiplier;
        POS_ACC = 3;
        SLOW_DIST = 5;
        driveTo(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY() + 20, Math.toRadians(90));
        customWait(() -> paused);
        driveTo(drive.getPoseEstimate().getX() + 2, drive.getPoseEstimate().getY() + 6, 0);
        SLOW_DIST = 20;
        POS_ACC = 1;
        intake.setIntakePower(0);

        distCorrect.sensor.setHighPass(false);
    }

    private void driveToWall() {
        H_ACC = Math.toRadians(5);
        POS_ACC = 2;
        SLOW_DIST = 10;
        SPEED_MULT = 0.8 * speedMultiplier;
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
        SPEED_MULT = speedMultiplier;
        MIN_TURN = 0.15;
    }

    private void driveIntoWarehouse() {
        //Start intaking
        armController.setScorePos(ArmController.ScoringPosition.IN);
        intake.setIntakePower(-0.5);

        POS_ACC = 2;
        H_ACC = Math.toRadians(20);

        //Drive through gap
        drive.setWeightedDrivePower(new Pose2d(0.55 * speedMultiplier, 0.45 * speedMultiplier, 0));
        long driveStartTime = System.currentTimeMillis();
        customWait(() -> {
            telemetry.addLine("DRIVE INTO WAREHOUSE");

            return System.currentTimeMillis() < driveStartTime + 600/speedMultiplier || distCorrect.getFrontDistance() > 45;
        });
        drive.setDrivePower(new Pose2d());

        intake.setIntakePower(1);

        //Distance correction
        long sensorStartTime = System.currentTimeMillis();
        customWait(() -> distCorrect.getFrontDistance() > 100 &&
                System.currentTimeMillis() < sensorStartTime + 500);
        if (distCorrect.getFrontDistance() < 60) {
            drive.setPoseEstimate(new Pose2d(
                    8.5,
                    144 - distCorrect.getFrontDistance(),
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
        drive.setWeightedDrivePower(new Pose2d(0.6 * speedMultiplier, 0.2 * speedMultiplier, 0));
        customWait(() -> {
            telemetry.addLine("INTAKE");

            if (distCorrect.getFrontDistance() < 60) {
                if (deltaHeading(drive.getPoseEstimate().getHeading(), Math.toRadians(95)) < 0)  {
                    drive.setWeightedDrivePower(new Pose2d(
                            speedMultiplier * Math.min(Math.pow(distCorrect.getFrontDistance()-5-distanceFromWall, 2) * 0.0002 + 0.125, 0.6), //Slow down as approaches wall
                            speedMultiplier * -0.15, //Drive away from wall
                            speedMultiplier * -0.2));
                }
                else if (deltaHeading(drive.getPoseEstimate().getHeading(), Math.toRadians(85)) > 0) {
                    drive.setWeightedDrivePower(new Pose2d(
                            speedMultiplier * Math.min(Math.pow(distCorrect.getFrontDistance()-5-distanceFromWall, 2) * 0.0002 + 0.125, 0.6), //Slow down as approaches wall
                            speedMultiplier * -0.15, //Drive away from wall
                            speedMultiplier * 0.2));
                }
                else {
                    drive.setWeightedDrivePower(new Pose2d(
                            speedMultiplier * Math.min(Math.pow(distCorrect.getFrontDistance()-5-distanceFromWall, 2) * 0.0002 + 0.125, 0.6), //Slow down as approaches wall
                            speedMultiplier * -0.15, //Drive away from wall
                            0));
                }
            }
            else {
                if (deltaHeading(drive.getPoseEstimate().getHeading(), Math.toRadians(95)) < 0)  {
                    drive.setWeightedDrivePower(new Pose2d(
                            speedMultiplier * 0.2, //Slow down as approaches wall
                            speedMultiplier * 0.2, //Drive away from wall
                            speedMultiplier * -0.2));
                }
                else if (deltaHeading(drive.getPoseEstimate().getHeading(), Math.toRadians(85)) > 0) {
                    drive.setWeightedDrivePower(new Pose2d(
                            speedMultiplier * 0.2, //Slow down as approaches wall
                            speedMultiplier * 0.2, //Drive away from wall
                            speedMultiplier * 0.2));
                }
                else {
                    drive.setWeightedDrivePower(new Pose2d(
                            speedMultiplier * 0.2, //Slow down as approaches wall
                            speedMultiplier * 0.2, //Drive away from wall
                            0));
                }
            }

            //Distance correction
            if (distCorrect.getFrontDistance() < 40) {
                drive.setPoseEstimate(new Pose2d(
                        10,
                        144 - distCorrect.getFrontDistance(),
                        drive.getPoseEstimate().getHeading()));
            }

            if (distCorrect.getFrontDistance() < 50 && intake.getFreightInIntake()) return false;
            if (System.currentTimeMillis() > driveStartTime + 1000/speedMultiplier) return false;

            if (System.currentTimeMillis() > driveStartTime + 500)
                return distCorrect.getFrontDistance() > distanceFromWall || distCorrect.getFrontDistance() > 100;

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

        //Make sure robot is against wall
        //kyle loves u.
        drive.setDrivePower(new Pose2d(speedMultiplier * -0.5, 0, speedMultiplier * -0.5));
        customWait(() -> distCorrect.getFrontDistance() < 50 &&
                drive.getPoseEstimate().getHeading() > Math.toRadians(94));
        drive.setWeightedDrivePower(new Pose2d(speedMultiplier * -0.3, speedMultiplier * 0.7, 0));
        waitForTime(150/speedMultiplier);

        //Distance correction
        long sensorStartTime = System.currentTimeMillis();
        customWait(() -> distCorrect.getFrontDistance() > 100 &&
                System.currentTimeMillis() < sensorStartTime + 750);
        if (distCorrect.getFrontDistance() < 50) {
            drive.setPoseEstimate(new Pose2d(
                    8.5,
                    144 - distCorrect.getFrontDistance(),
                    drive.getPoseEstimate().getHeading()));
        }

        drive.setWeightedDrivePower(new Pose2d(speedMultiplier * -0.55, speedMultiplier * 0.35, 0));

        AtomicBoolean throughGap = new AtomicBoolean(false);

        long driveStartTime = System.currentTimeMillis();

        customWait(() -> {
            telemetry.addLine("DRIVE OUT OF WAREHOUSE");

            if (distCorrect.getFrontDistance() > 70) {
                drive.setDrivePower(new Pose2d(speedMultiplier * -0.2, speedMultiplier * 0.2, 0));
                customWait(() -> distCorrect.getFrontDistance() > 100 &&
                        System.currentTimeMillis() < sensorStartTime + 250);
            }
            else
                drive.setWeightedDrivePower(new Pose2d(speedMultiplier * -0.55, speedMultiplier * 0.35, 0));

            if (distCorrect.getFrontDistance() > 40 && !throughGap.get()) {
                double robotHeading = drive.getPoseEstimate().getHeading();
                drive.setPoseEstimate(new Pose2d(7, 84, robotHeading));

                armController.setScorePos(ArmController.ScoringPosition.UP);

                throughGap.set(true);
            }

            //Distance correction
            if (distCorrect.getFrontDistance() < 70 && distCorrect.getFrontDistance() > 15) {
                drive.setPoseEstimate(new Pose2d(
                        6.5,
                        144 - distCorrect.getFrontDistance(),
                        drive.getPoseEstimate().getHeading()));
            }

            return System.currentTimeMillis() < driveStartTime + 1500/speedMultiplier &&
                    (distCorrect.getFrontDistance() < 45 || distCorrect.getFrontDistance() > 70);
        });
        drive.setDrivePower(new Pose2d());

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

        intake.setIntakePower(0);
    }

    private void scoreInHub(double yPos) {
        //Put arm up
        armController.setScorePos(ArmController.ScoringPosition.UP);

        SPEED_MULT = speedMultiplier;
        SLOW_DIST = 20;
        H_ACC = Math.toRadians(4);
        MIN_TURN = 0.25;

        long startTime = System.currentTimeMillis();

        AtomicBoolean armOut = new AtomicBoolean(false);

        //Drive to hub
        driveTo(() -> 11.5,
                () -> yPos,
                () -> {
                    //Wait for arm to get up before turning to hub
                    if (System.currentTimeMillis() < startTime + 50)
                        return Math.toRadians(10);
                    else if (!armOut.get()) {
                        armController.setScorePos(ArmController.ScoringPosition.HIGH);

                        armOut.set(true);

                        return Math.toRadians(-10);
                    }
                    else
                        return Math.toRadians(-10);
                },
                1500);

        H_ACC = Math.toRadians(1);
        SLOW_DIST = 20;
        SPEED_MULT = speedMultiplier;
        MIN_TURN = 0.15;

        commands.outtake(intake);
        waitForTime(150);
    }

    private void scoreInHub(double yPos, ArmController.ScoringPosition scorePos) {
        armController.setScorePos(scorePos);
        waitForTime(100);

        SPEED_MULT = speedMultiplier * 0.8;
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
                        return Math.toRadians(-25);
                    }
                },
                1500);

        H_ACC = Math.toRadians(1);
        SLOW_DIST = 20;
        SPEED_MULT = speedMultiplier;

        commands.outtake(intake);
        waitForTime(150);
    }
}
