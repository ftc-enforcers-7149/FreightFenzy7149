package org.firstinspires.ftc.teamcode.Autonomous.DuckSide;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.Auto_V2_5;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Levels;

@Autonomous(name = "Red Duck - Unit Park")
@Disabled
public class RedParkInUnit extends Auto_V2_5 {

    @Override
    protected Alliance getAlliance() {
        return Alliance.RED;
    }

    @Override
    protected void auto() {
        drive.setPoseEstimate(new Vector2d(0, -15));

        Levels liftHeight = commands.detectBarcode(tseDetector);

        intake.setIntakePower(-0.2);

        POS_ACC = 1;
        SLOW_DIST = 15;

        //Drive to the duckwheel
        driveTo(8, 13, 0);

        //Spin and stop duckwheel
        commands.spinDuck(spinner);

        //Set lift to correct level according to the vision
        lift.setTargetHeight(liftHeight);

        //Drive to hub
        driveTo(32,-29, Math.toRadians(320));

        //Drive to hub and outtake
        driveTo( 34,-32, Math.toRadians(320));
        commands.outtake(intake, lift);

        H_ACC = Math.toRadians(6);

        //Drive a little bit back and drop lift
        driveTo(31,-32, Math.toRadians(320));
        lift.setTargetHeight(Levels.GROUND);

        //Align with the warehouse and park
        driveTo(26,-20, Math.toRadians(270));
        driveTo(26,12, Math.toRadians(270));

        //Lower lift all the way down for TeleOp
        commands.setLiftHeight(lift, Levels.GROUND);
    }
}