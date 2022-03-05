package org.firstinspires.ftc.teamcode.Autonomous.DuckSide;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.Auto_V2_5;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.MotorIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Levels;

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
        drive.setPoseEstimate(new Vector2d(0, 15));

        Levels liftHeight = commands.detectBarcode(tseDetector);

        SLOW_DIST = 25;

        //Set lift to correct level according to the vision
        lift.setTargetHeight(liftHeight);

        //Drive to hub
        driveTo(27,24, Math.toRadians(50));

        //Drive to hub and outtake
        driveTo(31,29, Math.toRadians(50));
        commands.outtake(intake, lift);
        lift.setTargetHeight(Levels.GROUND);

        //Drive to the duckwheel
        POS_ACC = 3;
        driveTo(0, -14, Math.toRadians(90));
        POS_ACC = 1;

        //Spin and stop duckwheel
        commands.spinDuck(spinner);

        waitForTime(1000);

        //Try to intake duck
        intake.setIntakePower(0.5);
        driveTo(6, -14, Math.toRadians(160));
        driveTo(6, -6, Math.toRadians(160));
        driveTo(10, 9, Math.toRadians(90));
        intake.setIntakePower(0);
        intake.setLatch(MotorIntake.LatchPosition.DUCK_CLOSED);

        lift.setTargetHeight(Levels.HIGH);

        //Drive to hub
        driveTo(28,23, Math.toRadians(60));

        //Drive to hub and outtake
        driveTo( 32,28, Math.toRadians(60));
        commands.outtake(intake, lift);

        //Back away from hub
        driveTo(21,23, Math.toRadians(60));
        lift.setTargetHeight(Levels.GROUND);

        H_ACC = Math.toRadians(7);
        POS_ACC = 3;

        customWait(() -> (getRuntime() < 22));
        lift.setTargetHeight(Levels.LOW);

        //Align with the warehouse and park
        driveTo(24,33, Math.toRadians(270));
        driveTo(29,120, Math.toRadians(280));

        //Lower lift all the way down for TeleOp
        commands.setLiftHeight(lift, Levels.GROUND);

        driveTo(drive.getPoseEstimate().getX() - 5, drive.getPoseEstimate().getY(), 0);
    }
}