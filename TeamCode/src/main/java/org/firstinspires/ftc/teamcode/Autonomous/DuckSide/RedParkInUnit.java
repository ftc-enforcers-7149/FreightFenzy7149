package org.firstinspires.ftc.teamcode.Autonomous.DuckSide;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.Auto_V2;
import org.firstinspires.ftc.teamcode.Autonomous.HubLevel;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Lift;

@Autonomous(name = "Red Duck - Unit Park")
@Disabled
public class RedParkInUnit extends Auto_V2 {

    @Override
    protected Alliance getAlliance() {
        return Alliance.RED;
    }

    @Override
    protected void auto() {
        drive.setPoseEstimate(new Vector2d(0, -15));

        HubLevel liftHeight = commands.detectBarcode(tseDetector);

        intake.setIntakePower(-0.2);

        POS_ACC = 1;
        SLOW_DIST = 15;

        //Drive to the duckwheel
        driveTo(8, 12, 0);

        //Spin and stop duckwheel
        commands.spinDuck(spinner, 2750);

        //Set lift to correct level according to the vision
        switch (liftHeight) {
            case LOW:
                lift.setTargetHeight(Lift.LOW_HEIGHT + 1);
                break;
            case MIDDLE:
                lift.setTargetHeight(Lift.MIDDLE_HEIGHT);
                break;
            case HIGH:
                lift.setTargetHeight(Lift.HIGH_HEIGHT);
                break;
        }

        //Drive to hub
        driveTo(32,-29, Math.toRadians(320));

        //Drive to hub and outtake
        driveTo( 34,-32, Math.toRadians(320));
        commands.outtake(intake, 1250);

        H_ACC = Math.toRadians(6);

        //Drive a little bit back and drop lift
        driveTo(31,-32, Math.toRadians(320));
        lift.setTargetHeight(Lift.GROUND_HEIGHT);

        H_ACC = Math.toRadians(6);

        //Drive a little bit back and drop lift
        driveTo(31,-32, Math.toRadians(320));
        lift.setTargetHeight(Lift.GROUND_HEIGHT);

        //Align with the warehouse and park
        driveTo(26,-20, Math.toRadians(270));
        driveTo(26,12, Math.toRadians(270));

        //Lower lift all the way down for TeleOp
        commands.setLiftHeight(lift, Lift.GROUND_HEIGHT);
    }
}