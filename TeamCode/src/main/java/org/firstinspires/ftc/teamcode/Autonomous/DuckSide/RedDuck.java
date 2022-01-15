package org.firstinspires.ftc.teamcode.Autonomous.DuckSide;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.Auto_V2;
import org.firstinspires.ftc.teamcode.Autonomous.HubLevel;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Lift;

@Autonomous(name = "Red Duck")
//@Disabled
public class RedDuck extends Auto_V2 {

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
        driveTo(5, 6, 0);

        //Spin and stop duckwheel
        commands.spinDuck(spinner, 2750);

        //Drive to hub
        driveTo(33,-26, Math.toRadians(310));

        //Set lift to correct level according to the vision
        switch (liftHeight) {
            case LOW:
                commands.setLiftHeight(lift, Lift.LOW_HEIGHT);
                break;
            case MIDDLE:
                commands.setLiftHeight(lift, Lift.MIDDLE_HEIGHT);
                break;
            case HIGH:
                commands.setLiftHeight(lift, Lift.HIGH_HEIGHT - 2);
                break;
        }

        //Drive to hub and outtake
        driveTo( 35,-28, Math.toRadians(310));
        commands.outtake(intake, 1250);

        H_ACC = Math.toRadians(3);

        //Drive a little bit back and drop lift
        driveTo(33,-26, Math.toRadians(310));
        lift.setTargetHeight(Lift.GROUND_HEIGHT);

        customWait(() -> (getRuntime() < 24));
        lift.setTargetHeight(Lift.BARRIER_HEIGHT);

        //Align with the warehouse and park
        driveTo(20,-33, Math.toRadians(90));
        commands.setLiftHeight(lift, Lift.BARRIER_HEIGHT);

        SLOW_DIST = 20;
        POS_ACC = 3;
        driveTo(43,-120, Math.toRadians(100));

        //Lower lift all the way down for TeleOp
        commands.setLiftHeight(lift, Lift.GROUND_HEIGHT);

        driveTo(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), 0);
    }
}