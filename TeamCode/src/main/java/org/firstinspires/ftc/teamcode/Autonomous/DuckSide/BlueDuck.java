package org.firstinspires.ftc.teamcode.Autonomous.DuckSide;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.Auto_V2_5;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Levels;

@Autonomous(name = "Blue Duck - WH Park")
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

        intake.setIntakePower(-0.2);

        POS_ACC = 1;
        SLOW_DIST = 15;

        //Drive to the duckwheel
        driveTo(8, -12, 0);

        //Spin and stop duckwheel
        commands.spinDuck(spinner, 3000);

        //Set lift to correct level according to the vision
        lift.setTargetHeight(liftHeight);

        //Drive to hub
        driveTo(29,26, Math.toRadians(40));

        //Drive to hub and outtake
        driveTo(31,29, Math.toRadians(40));
        commands.outtake(intake, 1250);

        H_ACC = Math.toRadians(6);

        //Drive a little bit back and drop lift
        driveTo(27,29, Math.toRadians(40));
        lift.setTargetHeight(Levels.GROUND);

        customWait(() -> (getRuntime() < 24));
        lift.setTargetHeight(Levels.LOW);

        //Align with the warehouse and park
        driveTo(20,33, Math.toRadians(270));
        commands.setLiftHeight(lift, Levels.LOW);

        SLOW_DIST = 20;
        POS_ACC = 3;
        driveTo(20,120, Math.toRadians(280));

        //Lower lift all the way down for TeleOp
        commands.setLiftHeight(lift, Levels.GROUND);

        driveTo(drive.getPoseEstimate().getX() - 2, drive.getPoseEstimate().getY(), 0);
    }
}