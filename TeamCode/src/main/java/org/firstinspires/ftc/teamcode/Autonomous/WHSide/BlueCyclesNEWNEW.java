package org.firstinspires.ftc.teamcode.Autonomous.WHSide;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.Auto_V2_5;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.ArmController;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Levels;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Output;

import static org.firstinspires.ftc.teamcode.GlobalData.*;

@Autonomous(name = "Blue WH Cycles NEW NEW")
//@Disabled
public class BlueCyclesNEWNEW extends Auto_V2_5 {

    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }

    @Override
    protected void auto() {
        distCorrect.startRunning();

        //Initial setup
        drive.setPoseEstimate(new Pose2d(6.75, 78.25, Math.toRadians(0)));
        //Levels levels = commands.detectBarcode(tseDetector);
        Levels levels = Levels.HIGH;

        //Move arm up
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        armController.setScorePos(ArmController.ScoringPosition.UP);

        //Start driving towards hub as arm gets out of intake
        SPEED_MULT = 1;
        SLOW_DIST = 2;
        driveTo(13, 80, Math.toRadians(0));
        SPEED_MULT = 1;
        SLOW_DIST = 15;

        //Move arm out to score
        armController.setScorePos(ArmController.ScoringPosition.HIGH);

        //Drive rest of way to hub
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
        waitForTime(125);
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        SLOW_DIST = 15;

        drive.setPoseEstimate(new Pose2d(6.75, 84, Math.toRadians(0)));

        cycle();
        cycle();

        distCorrect.stopRunning();
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
