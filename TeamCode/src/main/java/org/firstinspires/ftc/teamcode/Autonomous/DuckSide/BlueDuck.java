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

import java.util.logging.Level;

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

        //Levels level = commands.detectBarcode(tseDetector);
        Levels level = Levels.HIGH;

        SLOW_DIST = 25;

        //Set lift to correct level according to the vision
        lift.setPower(1);
        waitForTime(250);

        switch (level) {
            case HIGH:
            default:
                armController.setScorePos(ArmController.ScoringPosition.HIGH);
                driveTo(16, 37, Math.toRadians(35));
                break;
            case MIDDLE:
                armController.setScorePos(ArmController.ScoringPosition.MIDDLE_AUTO);
                driveTo(16, 34, Math.toRadians(35));
                break;
            case LOW:
                armController.setScorePos(ArmController.ScoringPosition.LOW_AUTO);
                driveTo(16, 32, Math.toRadians(35));
                break;
        }

        commands.outtake(intake);

        //Drive to the duckwheel
        POS_ACC = 3;
        SPEED_MULT = 0.7;

        long driveStartTime = System.currentTimeMillis();
        driveTo(() -> 10d, () -> {
            if (System.currentTimeMillis() >= driveStartTime + 200)
                armController.setScorePos(ArmController.ScoringPosition.UP);

            return -4d;
        }, () -> Math.toRadians(90), 1500);
        SPEED_MULT = 1;
        POS_ACC = 1;

        //Spin and stop duckwheel
        armController.setScorePos(ArmController.ScoringPosition.IN);
        intake.setIntakePower(-0.5);
        commands.spinDuck(spinner);
        intake.setIntakePower(0.5);

        drive.setPoseEstimate(new Pose2d(12, 6.75, Math.toRadians(90)));

        waitForTime(750);

        //Try to intake duck
        SLOW_DIST = 10;
        POS_ACC = 2;
        intake.setIntakePower(0.5);
        driveTo(10, 12, Math.toRadians(160));
        driveTo(10, 22, Math.toRadians(160));
        driveTo(14, 28, Math.toRadians(90));
        intake.setIntakePower(0);
        intake.setLatch(MotorIntake.LatchPosition.DUCK_CLOSED);
        SLOW_DIST = 25;
        POS_ACC = 1;

        armController.setScorePos(ArmController.ScoringPosition.UP);

        //Drive to hub
        driveTo(12, 25, Math.toRadians(0));
        armController.setScorePos(ArmController.ScoringPosition.HIGH);

        //Drive to hub and outtake
        driveTo(15, 36, Math.toRadians(35));
        commands.outtake(intake);

        //Back away from hub
        driveTo(8,23, Math.toRadians(0));
        armController.setScorePos(ArmController.ScoringPosition.UP);

        //Park in storage unit
        driveTo(34, 0, Math.toRadians(0), 1500);
        armController.setScorePos(ArmController.ScoringPosition.IN);
        intake.setIntakePower(-1);
        waitForTime(400);
    }
}