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
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Levels;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Output;

import static org.firstinspires.ftc.teamcode.GlobalData.H_ACC;
import static org.firstinspires.ftc.teamcode.GlobalData.SLOW_DIST;
import static org.firstinspires.ftc.teamcode.GlobalData.SPEED_MULT;

@Autonomous(name = "Blue WH Cycles LIMITED")
@Disabled
public class BlueCyclesLIMITED extends Auto_V2_5 {

    private static final Trajectory driveToWall = MecanumDrive.trajectoryBuilder(new Pose2d(12, 78, Math.toRadians(-25)), Math.toRadians(170))
            .splineToSplineHeading(new Pose2d(-25, 80, Math.toRadians(100)), Math.toRadians(160))
            .addTemporalMarker(0.5, () -> GlobalData.armUpSignal = true)
            .build();

    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }

    @Override
    protected void auto() {
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
        waitForTime(500);

        switch (level) {
            case HIGH:
            default:
                scorePreload(77, ArmController.ScoringPosition.HIGH);
                break;
            case MIDDLE:
                scorePreload(74.5, ArmController.ScoringPosition.MIDDLE_AUTO);
                break;
            case LOW:
                scorePreload(71, ArmController.ScoringPosition.LOW_AUTO);
                break;
        }


    }

    private void scorePreload(double yPos, ArmController.ScoringPosition scorePos) {
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
