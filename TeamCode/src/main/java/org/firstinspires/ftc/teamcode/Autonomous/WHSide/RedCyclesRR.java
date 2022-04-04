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
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.MotorIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Output;

@Autonomous(name = "Red Cycles WH - RR")
//@Disabled
public class RedCyclesRR extends Auto_V2_5 {

    private static final Trajectory preload = MecanumDrive.trajectoryBuilder(new Pose2d(6.5, -65.5, Math.toRadians(90)), Math.toRadians(90))
            .splineToSplineHeading(new Pose2d(6, -44, Math.toRadians(145)), Math.toRadians(90))
            .addTemporalMarker(1.16, () -> GlobalData.openSignal = true) //Open latch just as arm lines up
            .addDisplacementMarker(() -> GlobalData.outtakeSignal = true) //Outtake freight right at the end
            .build();
    private static final Trajectory driveIn = MecanumDrive.trajectoryBuilder(new Pose2d(6, -44, Math.toRadians(145)), Math.toRadians(270))
            .splineToSplineHeading(new Pose2d(9, -58, Math.toRadians(90)), Math.toRadians(-60))
            .splineToConstantHeading(new Vector2d(16, -66), Math.toRadians(0))
            .addSpatialMarker(new Vector2d(24, -66), () -> GlobalData.armInSignal = true) //Bring arm to ground
            .splineToConstantHeading(new Vector2d(32, -66), Math.toRadians(0))
            .addDisplacementMarker(() -> GlobalData.intakeSignal = true) //Start intaking once in the warehouse
            .splineToSplineHeading(new Pose2d(48, -60, Math.toRadians(45)), Math.toRadians(30))
            .splineTo(new Vector2d(60, -60), Math.toRadians(-25))
            .build();
    private static final Trajectory driveOut = MecanumDrive.trajectoryBuilder(new Pose2d(60, -60, Math.toRadians(-15)), Math.toRadians(160))
            .splineTo(new Vector2d(48, -60), Math.toRadians(-160))
            .splineTo(new Vector2d(38, -64), Math.toRadians(-165))
            .addDisplacementMarker(() -> GlobalData.armUpSignal = true) //Bring arm up while going through gap
            .splineTo(new Vector2d(28, -66), Math.toRadians(-175))
            .splineTo(new Vector2d(19, -66), Math.toRadians(-185))
            .splineToSplineHeading(new Pose2d(6, -44, Math.toRadians(145)), Math.toRadians(90))
            .addTemporalMarker(2.0, () -> GlobalData.armOutSignal = true) //Start moving arm out as late as possible
            .addTemporalMarker(2.54, () -> GlobalData.openSignal = true) //Open latch just as arm lines up
            .addDisplacementMarker(() -> GlobalData.outtakeSignal = true) //Outtake freight right at the end
            .build();
    private static final Trajectory park = MecanumDrive.trajectoryBuilder(new Pose2d(6, -44, Math.toRadians(145)), Math.toRadians(270))
            .splineToSplineHeading(new Pose2d(9, -58, Math.toRadians(90)), Math.toRadians(-60))
            .splineToConstantHeading(new Vector2d(16, -66), Math.toRadians(0))
            .addSpatialMarker(new Vector2d(24, -66), () -> GlobalData.armInSignal = true) //Bring arm to ground
            .splineToConstantHeading(new Vector2d(36, -66), Math.toRadians(0))
            .build();

    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }

    @Override
    protected void auto() {
        distCorrect.startRunning();

        //Initial setup
        drive.setPoseEstimate(new Pose2d(6.5, -65.5, Math.toRadians(90)));
        //Levels levels = commands.detectBarcode(tseDetector);
        ArmController.ScoringPosition scorePos = ArmController.ScoringPosition.HIGH;

        //Extend arm and score preloaded block
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        Output extendArm = new Output() {
            long startTime = 0;

            @Override
            public void updateOutput() {
                if (lift.getHeight() >= ArmController.ScoringPosition.UP.liftPos - 0.5)
                    stopOutput();
            }

            @Override
            public void startOutput() {
                startTime = System.currentTimeMillis();
                armController.setScorePos(ArmController.ScoringPosition.UP);
            }

            @Override
            public void stopOutput() {
                armController.setScorePos(scorePos);
                removeOutput(this);
            }
        };
        addOutput(extendArm);
        extendArm.startOutput();
        drive.followTrajectoryAsync(preload);
        waitForDriveComplete();

        for (int i = 0; i < 4; i++) {
            //Retract arm, drive into warehouse, and intake freight
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            Output retractArm = new Output() {
                long startTime = 0;

                @Override
                public void updateOutput() {
                    if (System.currentTimeMillis() > startTime + 200)
                        stopOutput();
                }

                @Override
                public void startOutput() {
                    startTime = System.currentTimeMillis();
                }

                @Override
                public void stopOutput() {
                    armController.setScorePos(ArmController.ScoringPosition.UP);
                    removeOutput(this);
                }
            };
            addOutput(retractArm);
            retractArm.startOutput();
            drive.followTrajectoryAsync(driveIn);
            waitForDriveComplete();
            intake.setIntakePower(0);
            intake.setLatch(MotorIntake.LatchPosition.CLOSED);

            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            drive.followTrajectoryAsync(driveOut);
            waitForDriveComplete();
        }

        //Retract arm, drive into warehouse, and park
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
        Output retractArm = new Output() {
            long startTime = 0;

            @Override
            public void updateOutput() {
                if (System.currentTimeMillis() > startTime + 200)
                    stopOutput();
            }

            @Override
            public void startOutput() {
                startTime = System.currentTimeMillis();
            }

            @Override
            public void stopOutput() {
                armController.setScorePos(ArmController.ScoringPosition.UP);
                removeOutput(this);
            }
        };
        addOutput(retractArm);
        retractArm.startOutput();
        drive.followTrajectoryAsync(park);
        waitForDriveComplete();
    }
}
