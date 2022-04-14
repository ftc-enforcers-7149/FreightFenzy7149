package org.firstinspires.ftc.teamcode.Autonomous.DuckSide;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.Auto_V2_5;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.ArmController;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.MotorIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Levels;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Output;

import static org.firstinspires.ftc.teamcode.GlobalData.*;

@Autonomous(name = "Blue Duck")
//@Disabled
public class BlueDuck extends Auto_V2_5 {

    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }

    @Override
    protected void auto() {
        drive.setPoseEstimate(new Pose2d(6.75, 30.5, Math.toRadians(0)));

        Levels level = commands.detectBarcode(tseDetector);
        //Levels level = Levels.HIGH;

        SLOW_DIST = 15;
        MIN_TURN = 0.2;

        //Set lift to correct level according to the vision
        lift.setPower(1);
        waitForTime(500);
        fourBar.setPosition(ArmController.ScoringPosition.HIGH_AUTO.barPos);
        waitForTime(250);

        driveTo(10, 32, Math.toRadians(10));

        switch (level) {
            case HIGH:
            default:
                armController.setScorePos(ArmController.ScoringPosition.HIGH_AUTO);
                driveTo(15, 40, Math.toRadians(35));
                break;
            case MIDDLE:
                armController.setScorePos(ArmController.ScoringPosition.MIDDLE_AUTO);
                driveTo(15, 42, Math.toRadians(35));
                break;
            case LOW:
                armController.setScorePos(ArmController.ScoringPosition.LOW_AUTO);
                driveTo(17, 44.5, Math.toRadians(38));
                break;
        }

        commands.outtake(intake);
        waitForTime(200);

        //Drive to the duckwheel
        POS_ACC = 1.5;
        SPEED_MULT = 0.225;
        SLOW_DIST = 20;

        long driveStartTime = System.currentTimeMillis();
        driveTo(() -> 8d, () -> {
            if (System.currentTimeMillis() >= driveStartTime + 400)
                armController.setScorePos(ArmController.ScoringPosition.UP);

            return -6d;
        }, () -> Math.toRadians(90), 1500);
        SPEED_MULT = 1;
        driveTo(5, drive.getPoseEstimate().getY(), Math.toRadians(90), 600);
        SPEED_MULT = 1;
        POS_ACC = 1;
        SLOW_DIST = 15;

        //Spin and stop duckwheel
        armController.setScorePos(ArmController.ScoringPosition.IN);
        intake.setIntakePower(-0.5);
        commands.spinDuck(spinner);
        intake.setIntakePower(0.75);

        drive.setPoseEstimate(new Pose2d(12, 6.75, drive.getPoseEstimate().getHeading()));

        waitForTime(750);

        //Try to intake duck
        SLOW_DIST = 5;
        POS_ACC = 2;
        intake.setIntakePower(1);
        driveTo(12.5, 10, Math.toRadians(180));
        driveTo(12, 30, Math.toRadians(160));
        driveTo(15.5, 36, Math.toRadians(90));
        intake.setIntakePower(0);
        intake.setLatch(MotorIntake.LatchPosition.DUCK_CLOSED);
        SLOW_DIST = 15;
        POS_ACC = 1;

        lift.setPower(1);
        long startLiftTime = System.currentTimeMillis();

        //Drive to hub
        driveTo(12, 33, Math.toRadians(32));
        customWait(() -> System.currentTimeMillis() <= startLiftTime + 500);
        fourBar.setPosition(ArmController.ScoringPosition.HIGH_AUTO.barPos);
        waitForTime(250);

        armController.setScorePos(ArmController.ScoringPosition.HIGH_AUTO);

        //Drive to hub and outtake duck
        driveTo(16, 41, Math.toRadians(32));
        intake.setLatch(MotorIntake.LatchPosition.OPEN_UP);
        waitForTime(200);
        intake.setPaddle(MotorIntake.PaddlePosition.OUT_FAR);
        waitForTime(200);

        //Back away from hub
        driveTo(15, 23, Math.toRadians(0));
        armController.setScorePos(ArmController.ScoringPosition.UP);

        //Park in storage unit
        driveTo(36, 0, Math.toRadians(0), 1500);
        armController.setScorePos(ArmController.ScoringPosition.IN);
        intake.setIntakePower(-1);
        waitForTime(400);
    }
}