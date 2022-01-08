package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.Auto_V2;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Lift;

import java.util.function.Supplier;

@Autonomous(name = "Test Red Cycles")
//@Disabled
public class RedCycles extends Auto_V2 {

    @Override
    protected Alliance getAlliance() {
        return Alliance.RED;
    }

    @Override
    public void auto() {
        drive.setPoseEstimate(new Pose2d(6.75, -65.75, 0));

        POS_ACC = 0.5;

        //Pre-Setup
        driveTo(30.75, -65.75, Math.toRadians(45));

        //Align with wall
        lift.setTargetHeight(Lift.LOW_HEIGHT);
        waitForTime(2000);
        driveToWall();

        //Drive through gap
        lift.setTargetHeight(Lift.GROUND_HEIGHT);
        intake.setIntakePower(-1);
        driveIntoWarehouse();

        //Intake / Don't hit wall
        intake();
    }

    private void driveToWall() {
        distCorrect.startRunning();

        H_ACC = Math.toRadians(3);
        POS_ACC = 1.5;

        driveTo(() -> {
                    if (deltaHeading(drive.getPoseEstimate().getHeading(), Math.toRadians(270)) > Math.toRadians(3))
                        return 6.5;
                    else
                        return drive.getPoseEstimate().getX() - (distCorrect.getSideWall() - 6.75);
                },
                () -> -80.0,
                () -> Math.toRadians(270)
        );

        drive.setPoseEstimate(new Pose2d(
                distCorrect.getSideWall(),
                drive.getPoseEstimate().getY(),
                Math.toRadians(270)));

        H_ACC = Math.toRadians(1);
        POS_ACC = 0.5;

        distCorrect.stopRunning();
    }

    private void driveIntoWarehouse() {
        distCorrect.startRunning();

        driveTo(() -> drive.getPoseEstimate().getX() - (distCorrect.getSideWall() - 6.75),
                () -> -120.0,
                () -> Math.toRadians(270)
        );

        drive.setPoseEstimate(new Pose2d(distCorrect.correctPoseWithDist(), Math.toRadians(270)));

        distCorrect.stopRunning();
    }

    private void intake() {
        distCorrect.startRunning();

        lift.setTargetHeight(Lift.GROUND_HEIGHT);
        intake.setIntakePower(-1);

        fLeft.setPower(0.4);
        fRight.setPower(0.4);
        bLeft.setPower(0.4);
        bRight.setPower(0.4);

        while (opModeIsActive() &&
                !intake.getFreightInIntake() &&
                distCorrect.getFrontDistance() > 10) {
            updateInputs();
            updateOutputs();
        }

        fLeft.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);
    }
}
