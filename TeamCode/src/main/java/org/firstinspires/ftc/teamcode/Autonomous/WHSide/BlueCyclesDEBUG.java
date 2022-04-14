package org.firstinspires.ftc.teamcode.Autonomous.WHSide;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

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
@Disabled
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

    private boolean sensorState = true;

    private boolean paused = false;
    private boolean x, lastX;

    private double speedMultiplier = 1.0;
    private boolean dUp, dDown, lastDUp, lastDDown;

    private boolean reset = false;
    private boolean b, lastB;

    private boolean park = false;
    private boolean y, lastY;

    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }

    @Override
    public void auto() {
        distCorrect.startRunning();
        //distCorrect.sensor.setHighPass(true, 200);
        distCorrect.sensor.disableMoving();

        //Debug distance and intake sensors with leds
        //RED: no freight
        //YELLOW: freight
        //SOLID: good sensors
        //STROBE: bad sensors
        addOutput(new Output() {
            @Override
            public void updateOutput() {
                if (sensorState) {
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
                b = gamepad1.b;
                y = gamepad1.y;

                if (x && !lastX)
                    paused = !paused;

                if (paused) intake.setIntakePower(0);

                if (b && !lastB) {
                    paused = true;
                    reset = true;
                }

                if (y && !lastY) {
                    park = true;
                    reset = true;
                    paused = true;
                }

                lastX = x;
                lastB = b;
                lastY = y;
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

        while (opModeIsActive()) {
            //Align with warehouse
            //led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            if (!reset) {
                customWait(() -> paused);
                driveToWall();
            }

            //Drive through gap
            reset = false;
            customWait(() -> paused);
            driveIntoWarehouse();

            if (park)
                break;

            //Intake and don't hit wall
            if (!reset) {
                customWait(() -> paused);
                intake(20);
            }

            //Drive out through gap
            //led.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            if (!reset) {
                customWait(() -> paused);
                driveOutOfWarehouse();
            }

            //Drive to and score in hub
            //led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            if (!reset) {
                customWait(() -> paused);
                scoreInHub(62);
            }
        }

        //Park

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

            if (distCorrect.getFrontDistance() > 120)
                drive.setWeightedDrivePower(new Pose2d(0.55 * speedMultiplier, 0.45 * speedMultiplier, 0).times(0.5));
            else
                drive.setWeightedDrivePower(new Pose2d(0.55 * speedMultiplier, 0.45 * speedMultiplier, 0));
            return System.currentTimeMillis() < driveStartTime + 600/speedMultiplier || distCorrect.getFrontDistance() > 45;
        });
        drive.setWeightedDrivePower(new Pose2d());

        intake.setIntakePower(1);

        //Distance correction
        long sensorStartTime = System.currentTimeMillis();
        customWait(() -> {
            if (distCorrect.getFrontDistance() > 100 &&
                    System.currentTimeMillis() <= sensorStartTime + 500) {
                sensorState = false;
                return true;
            }
            if (System.currentTimeMillis() <= sensorStartTime + 500) sensorState = true;
            return false;
        });
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
                sensorState = true;
                if (deltaHeading(drive.getPoseEstimate().getHeading(), Math.toRadians(95)) < 0)  {
                    drive.setWeightedDrivePower(new Pose2d(
                            speedMultiplier * Math.max(Math.min(Math.pow(distCorrect.getFrontDistance()-distanceFromWall, 2) * 0.0002 + 0.125, 0.6), 0.15), //Slow down as approaches wall
                            speedMultiplier * -0.2, //Drive away from wall
                            speedMultiplier * -0.1));
                }
                else if (deltaHeading(drive.getPoseEstimate().getHeading(), Math.toRadians(85)) > 0) {
                    drive.setWeightedDrivePower(new Pose2d(
                            speedMultiplier * Math.max(Math.min(Math.pow(distCorrect.getFrontDistance()-distanceFromWall, 2) * 0.0002 + 0.125, 0.6), 0.15), //Slow down as approaches wall
                            speedMultiplier * -0.2, //Drive away from wall
                            speedMultiplier * 0.1));
                }
                else {
                    drive.setWeightedDrivePower(new Pose2d(
                            speedMultiplier * Math.max(Math.min(Math.pow(distCorrect.getFrontDistance()-distanceFromWall, 2) * 0.0002 + 0.125, 0.6), 0.15), //Slow down as approaches wall
                            speedMultiplier * -0.2, //Drive away from wall
                            0));
                }
            }
            else {
                sensorState = false;
                if (deltaHeading(drive.getPoseEstimate().getHeading(), Math.toRadians(95)) < 0)  {
                    drive.setWeightedDrivePower(new Pose2d(
                            speedMultiplier * 0.15, //Slow down as approaches wall
                            speedMultiplier * -0.15, //Drive away from wall
                            speedMultiplier * -0.1));
                }
                else if (deltaHeading(drive.getPoseEstimate().getHeading(), Math.toRadians(85)) > 0) {
                    drive.setWeightedDrivePower(new Pose2d(
                            speedMultiplier * 0.15, //Slow down as approaches wall
                            speedMultiplier * -0.15, //Drive away from wall
                            speedMultiplier * 0.1));
                }
                else {
                    drive.setWeightedDrivePower(new Pose2d(
                            speedMultiplier * 0.15, //Slow down as approaches wall
                            speedMultiplier * -0.15, //Drive away from wall
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
        drive.setWeightedDrivePower(new Pose2d(speedMultiplier * -0.5, 0, speedMultiplier * -0.5));
        customWait(() -> drive.getPoseEstimate().getHeading() > Math.toRadians(94));
        drive.setWeightedDrivePower(new Pose2d(speedMultiplier * -0.3, speedMultiplier * 0.6, speedMultiplier * 0.1));
        waitForTime(150/speedMultiplier);

        //Distance correction
        long sensorStartTime = System.currentTimeMillis();
        customWait(() -> {
            if (distCorrect.getFrontDistance() > 100 &&
                    System.currentTimeMillis() <= sensorStartTime + 750) {
                drive.setWeightedDrivePower(new Pose2d());
                sensorState = false;
                return true;
            }
            if (System.currentTimeMillis() <= sensorStartTime + 750) sensorState = true;
            return false;
        });
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
                sensorState = false;
                drive.setWeightedDrivePower(new Pose2d(speedMultiplier * -0.2, speedMultiplier * 0.2, 0));
                customWait(() -> {
                    if (distCorrect.getFrontDistance() > 100 &&
                            System.currentTimeMillis() <= sensorStartTime + 250) {
                        sensorState = false;
                        return true;
                    }
                    if (System.currentTimeMillis() <= sensorStartTime + 250) sensorState = true;
                    return false;
                });
            }
            else {
                sensorState = true;
                drive.setWeightedDrivePower(new Pose2d(speedMultiplier * -0.55, speedMultiplier * 0.35, 0));
            }

            if (distCorrect.getFrontDistance() > 40 && distCorrect.getFrontDistance() < 120 && !throughGap.get()) {
                double robotHeading = drive.getPoseEstimate().getHeading();
                //drive.setPoseEstimate(new Pose2d(7, 84, robotHeading));

                armController.setScorePos(ArmController.ScoringPosition.UP);

                throughGap.set(true);
            }

            //Distance correction
//            if (distCorrect.getFrontDistance() < 70 && distCorrect.getFrontDistance() > 15) {
//                drive.setPoseEstimate(new Pose2d(
//                        6.5,
//                        144 - distCorrect.getFrontDistance(),
//                        drive.getPoseEstimate().getHeading()));
//            }

            return System.currentTimeMillis() < driveStartTime + 1500/speedMultiplier &&
                    drive.getPoseEstimate().getY() > 90;

            //return System.currentTimeMillis() < driveStartTime + 1500/speedMultiplier &&
            //        (distCorrect.getFrontDistance() < 45 || distCorrect.getFrontDistance() > 70);
        });
        drive.setWeightedDrivePower(new Pose2d());

        //Distance correction
        //Distance correction
        long sensorStartTimeAfter = System.currentTimeMillis();
        customWait(() -> {
            if (distCorrect.getFrontDistance() > 100 &&
                    System.currentTimeMillis() <= sensorStartTimeAfter + 750) {
                sensorState = false;
                return true;
            }
            if (System.currentTimeMillis() <= sensorStartTime + 750) sensorState = true;
            return false;
        });
        if (distCorrect.getFrontDistance() < 100 && distCorrect.getFrontDistance() > 15) {
            sensorState = true;
//            drive.setPoseEstimate(new Pose2d(
//                    6.5,
//                    144 - distCorrect.getFrontDistance(),
//                    drive.getPoseEstimate().getHeading()));
        }
        else
            sensorState = false;

        drive.setPoseEstimate(new Pose2d(6.5, drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading()));

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
