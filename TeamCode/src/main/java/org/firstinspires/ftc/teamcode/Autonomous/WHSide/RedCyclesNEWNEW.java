package org.firstinspires.ftc.teamcode.Autonomous.WHSide;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.Auto_V2_5;
import org.firstinspires.ftc.teamcode.GlobalData;
import org.firstinspires.ftc.teamcode.Odometry.DriveWheels.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.ArmController;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Output;

import static org.firstinspires.ftc.teamcode.GlobalData.H_ACC;
import static org.firstinspires.ftc.teamcode.GlobalData.MIN_TURN;
import static org.firstinspires.ftc.teamcode.GlobalData.POS_ACC;
import static org.firstinspires.ftc.teamcode.GlobalData.SLOW_DIST;
import static org.firstinspires.ftc.teamcode.GlobalData.SPEED_MULT;

@Autonomous(name = "Red WH Cycles NEW NEW")
//@Disabled
public class RedCyclesNEWNEW extends Auto_V2_5 {

    private static final Trajectory preload = MecanumDrive.trajectoryBuilder(new Pose2d(6.75, -78.25, Math.toRadians(0)), Math.toRadians(-10))
            .splineToSplineHeading(new Pose2d(20, -78, Math.toRadians(25)), Math.toRadians(10))
            .addTemporalMarker(0.67, () -> GlobalData.openSignal = true) //Open latch just as arm lines up
            .addDisplacementMarker(() -> GlobalData.outtakeSignal = true) //Outtake freight right at the end
            .build();

    private static final Trajectory driveToWall = MecanumDrive.trajectoryBuilder(preload.end(), Math.toRadians(-170))
            .splineToSplineHeading(new Pose2d(4, -84, Math.toRadians(-75)), Math.toRadians(-170))
            .addTemporalMarker(0.5, () -> GlobalData.armUpSignal = true)
            .build();

    private static final Trajectory driveOut = MecanumDrive.trajectoryBuilder(new Pose2d(12, -130, Math.toRadians(-90)), Math.toRadians(110))
            .splineTo(new Vector2d(7, -120), Math.toRadians(120))
            .addTemporalMarker(0.6, () ->  GlobalData.armUpSignal = true)
            .splineToSplineHeading(new Pose2d(2, -95, Math.toRadians(-90)), Math.toRadians(90))
            .splineTo(new Vector2d(2, -80), Math.toRadians(90))
            .build();

    private static final Trajectory score = MecanumDrive.trajectoryBuilder(new Pose2d(6.5, -80, Math.toRadians(-90)), Math.toRadians(0))
            .splineToSplineHeading(new Pose2d(20, -78, Math.toRadians(25)), Math.toRadians(25))
            .addTemporalMarker(0.77, () -> GlobalData.openSignal = true) //Open latch just as arm lines up
            .addDisplacementMarker(() -> GlobalData.outtakeSignal = true) //Outtake freight right at the end
            .build();

    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }

    @Override
    protected void auto() {
        distCorrect.startRunning();
        intake.startScanningIntake();

        //Initial setup
        drive.setPoseEstimate(new Pose2d(6.75, -78.25, Math.toRadians(0)));
        //Levels levels = commands.detectBarcode(tseDetector);
        ArmController.ScoringPosition scorePos = ArmController.ScoringPosition.HIGH;

        preloaded(scorePos);
        driveIn();

        for (int i = 0; i < 5; i++) {
            driveOut();
            score();
            driveIn();
        }

        //Start driving to wall
        /*SLOW_DIST = 1;
        driveTo(5, 84, Math.toRadians(0));
        armController.setScorePos(ArmController.ScoringPosition.UP); //Bring arm in
        drive.setWeightedDrivePower(new Pose2d(-1, 0, 0));
        waitForTime(125);
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        SLOW_DIST = 15;*/

        distCorrect.stopRunning();
        intake.stopScanningIntake();
    }

    private void preloaded(ArmController.ScoringPosition scorePos) {
        //Extend arm and score preloaded block
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        Output extendArm = new Output() {
            long startTime = 0;

            @Override
            public void updateOutput() {
                if (lift.getHeight() >= ArmController.ScoringPosition.UP.liftPos - 4)
                    stopOutput();
            }

            @Override
            public void startOutput() {
                startTime = System.currentTimeMillis();
                lift.setPower(1);
            }

            @Override
            public void stopOutput() {
                armController.setScorePos(scorePos);
                removeOutput(this);
            }
        };
        addOutput(extendArm);
        extendArm.startOutput();
        waitForTime(150);
        drive.followTrajectoryAsync(preload);
        waitForDriveComplete();
    }

    private void driveIn() {
        //Drive to wall
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        drive.followTrajectoryAsync(driveToWall);
        waitForDriveComplete();

        //Drive into warehouse TODO: 0.458 seconds to do this, currently
        drive.setDrivePower(new Pose2d(0.8, -0.2, 0));
        long driveStartTime = System.currentTimeMillis();
        customWait(() -> {
            if (distCorrect.getFrontDistance() < 50)
                drive.setWeightedDrivePower(new Pose2d(
                        Math.min(Math.pow(distCorrect.getFrontDistance()-15, 4) * 0.000001, 0.8), //Slow down as approaches wall
                        0, //Drive away from wall
                        0));

            //Distance correction
            if (distCorrect.getFrontDistance() < 70 && distCorrect.getFrontDistance() > 25) {
                drive.setPoseEstimate(new Pose2d(
                        6.5,
                        distCorrect.getFrontDistance() - 144,
                        Math.toRadians(-90)));
            }

            return ((!intake.getFreightInIntake() &&
                    distCorrect.getFrontDistance() > 11) ||
                    distCorrect.getFrontDistance() > 45) &&
                    System.currentTimeMillis() < driveStartTime + 1000;
        });
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
    }

    private void driveOut() {
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

        intake.spitOutTwo();
        drive.followTrajectoryAsync(driveOut);
        waitForDriveComplete();

        if (distCorrect.getFrontDistance() < 90 && distCorrect.getFrontDistance() > 25) {
            drive.setPoseEstimate(new Pose2d(
                    6.5,
                    distCorrect.getFrontDistance() - 144,
                    Math.toRadians(-90)));
        }
    }

    private void score() {
        Output extendArm = new Output() {
            long startTime = 0;

            @Override
            public void updateOutput() {
                if (lift.getHeight() >= ArmController.ScoringPosition.UP.liftPos - 4)
                    stopOutput();
            }

            @Override
            public void startOutput() {
                startTime = System.currentTimeMillis();
                lift.setPower(1);
            }

            @Override
            public void stopOutput() {
                armController.setScorePos(ArmController.ScoringPosition.HIGH);
                removeOutput(this);
            }
        };
        addOutput(extendArm);
        extendArm.startOutput();
        drive.followTrajectoryAsync(score);
        waitForDriveComplete();
    }

    private void cycle() {
        //Strafe into warehouse
        SLOW_DIST = 1;
        POS_ACC = 2;
        H_ACC = Math.toRadians(3);
        driveTo(6, 110, Math.toRadians(0));
        SLOW_DIST = 15;
        POS_ACC = 1;
        H_ACC = Math.toRadians(1);
        armController.setScorePos(ArmController.ScoringPosition.IN); //Bring arm to ground
        intake.setIntakePower(-1);

        //Automatically start intaking after 150ms
        Output intaking = new Output() {
            long startTime = 0;

            @Override
            public void updateOutput() {
                if (System.currentTimeMillis() >= startTime + 150)
                    stopOutput();
            }

            @Override
            public void startOutput() {
                startTime = System.currentTimeMillis();
            }

            @Override
            public void stopOutput() {
                intake.setIntakePower(1);
                removeOutput(this);
            }
        };
        addOutput(intaking);
        intaking.startOutput();

        //Turn towards pile
        H_ACC = Math.toRadians(5);
        MIN_TURN = 0.3;
        driveTo(8, 120, Math.toRadians(60));
        H_ACC = Math.toRadians(1);
        MIN_TURN = 0.2;

        //Intake freight
        SLOW_DIST = 5;
        driveTo(10, 144, Math.toRadians(75), 500);
        SLOW_DIST = 15;

        //Stop intaking and bring up arm
        commands.closeIntake(intake);
        armController.setScorePos(ArmController.ScoringPosition.UP);

        //Drive back through gap
        drive.setDrivePower(new Pose2d(-0.4, 0.6, 0));
        waitForTime(500);
        drive.setDrivePower(new Pose2d(-0.95, 0.05, 0));
        waitForTime(900);

        drive.setPoseEstimate(new Pose2d(
                6.5,
                144-distCorrect.getFrontDistance()-3,
                Math.toRadians(90)
        ));

        //Move arm out to score automatically
        Output moveArmOut = new Output() {
            long startTime = 0;

            @Override
            public void updateOutput() {
                if (System.currentTimeMillis() >= startTime + 300)
                    stopOutput();
            }

            @Override
            public void startOutput() {
                startTime = System.currentTimeMillis();
            }

            @Override
            public void stopOutput() {
                armController.setScorePos(ArmController.ScoringPosition.HIGH);
                removeOutput(this);
            }
        };
        addOutput(moveArmOut);
        moveArmOut.startOutput();

        //Drive to hub
        SPEED_MULT = 0.225;
        MIN_TURN = 0.25;
        H_ACC = Math.toRadians(3);
        driveTo(17, 82, Math.toRadians(325));
        SPEED_MULT = 1;
        MIN_TURN = 0.15;
        H_ACC = Math.toRadians(1);

        //Score
        commands.outtake(intake);

        //Start driving to wall
        SLOW_DIST = 1;
        driveTo(5, 84, Math.toRadians(0));
        armController.setScorePos(ArmController.ScoringPosition.UP); //Bring arm in
        drive.setWeightedDrivePower(new Pose2d(-1, 0, 0));
        waitForTime(150);
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        SLOW_DIST = 15;

        //Reset position for next cycle
        drive.setPoseEstimate(new Pose2d(6.75, 84, Math.toRadians(0)));
    }
}