package org.firstinspires.ftc.teamcode.Autonomous.DuckSide;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.Auto_V2_5;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.MotorIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Levels;

import static org.firstinspires.ftc.teamcode.GlobalData.*;

@Autonomous(name = "Red Duck")
//@Disabled
public class RedDuck extends Auto_V2_5 {

    @Override
    protected Alliance getAlliance() {
        return Alliance.RED;
    }

    @Override
    protected void auto() {
        drive.setPoseEstimate(new Vector2d(0, -15));

        Levels liftHeight = commands.detectBarcode(tseDetector);

        SLOW_DIST = 25;

        //Set lift to correct level according to the vision
        lift.setTargetHeight(liftHeight);

        //Drive to hub
        driveTo(26,-23, Math.toRadians(310));

        //Drive to hub and outtake
        driveTo( 30,-28, Math.toRadians(310));
        commands.outtake(intake, lift);
        lift.setTargetHeight(Levels.GROUND);

        //Drive to the duckwheel
        POS_ACC = 3;
        driveTo(3, 14, Math.toRadians(270));
        POS_ACC = 1;

        //Spin and stop duckwheel
        commands.spinDuck(spinner);

        waitForTime(1000);

        //Try to intake duck
        intake.setIntakePower(0.5);
        driveTo(6, 14, Math.toRadians(210));
        driveTo(6, 6, Math.toRadians(210));
        driveTo(10, -9, Math.toRadians(270));
        intake.setIntakePower(0);
        intake.setLatch(MotorIntake.LatchPosition.DUCK_CLOSED);

        lift.setTargetHeight(Levels.HIGH);

        //Drive to hub
        driveTo(26,-23, Math.toRadians(320));

        //Drive to hub and outtake
        driveTo( 30,-28, Math.toRadians(320));
        commands.outtake(intake, lift);

        //Back away from hub
        driveTo(19,-23, Math.toRadians(320));
        lift.setTargetHeight(Levels.GROUND);

        H_ACC = Math.toRadians(7);
        POS_ACC = 3;

        customWait(() -> (getRuntime() < 22));
        lift.setTargetHeight(Levels.LOW);

        //Align with the warehouse and park
        driveTo(13,-33, Math.toRadians(90));
        driveTo(18,-120, Math.toRadians(100));

        //Lower lift all the way down for TeleOp
        commands.setLiftHeight(lift, Levels.GROUND);

        driveTo(drive.getPoseEstimate().getX() - 5, drive.getPoseEstimate().getY(), 0);
    }
}