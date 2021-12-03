package org.firstinspires.ftc.teamcode.Autonomous.DuckSide;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.Auto_V2;
import org.firstinspires.ftc.teamcode.Autonomous.HubLevel;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Lift;

@Autonomous(name = "Blue Duck")
//@Disabled
public class BlueDuck extends Auto_V2 {

    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }

    @Override
    protected void auto() {
        HubLevel liftHeight = commands.detectBarcode(tseDetector);

        POS_ACC = 1;
        SLOW_DIST = 15;

        //Drive to the duckwheel
        driveTo(5, -4, 0);

        //Spin and stop duckwheel
        commands.spinDuck(spinner, 2500);

        //Drive to hub
        driveTo(32,25, Math.toRadians(60));

        //Set lift to correct level according to the vision
        switch (liftHeight) {
            case LOW:
                commands.setLiftHeight(lift, Lift.LOW_HEIGHT);
                break;
            case MIDDLE:
                commands.setLiftHeight(lift, Lift.MIDDLE_HEIGHT);
                break;
            case HIGH:
                commands.setLiftHeight(lift, Lift.HIGH_HEIGHT);
                break;
        }

        //Drive to hub and outtake
        driveTo(34,27, Math.toRadians(70));
        commands.outtake(intake);

        H_ACC = Math.toRadians(6);

        //Drive a little bit back and drop lift
        driveTo(32,25, Math.toRadians(70));
        lift.setTargetHeight(Lift.GROUND_HEIGHT);

        customWait(() -> (getRuntime() < 24));
        lift.setTargetHeight(Lift.BARRIER_HEIGHT);

        //Align with the warehouse and park
        driveTo(20,33, Math.toRadians(270));
        commands.setLiftHeight(lift, Lift.BARRIER_HEIGHT);

        SLOW_DIST = 20;
        driveTo(15,128, Math.toRadians(270));

        //Lower lift all the way down for TeleOp
        commands.setLiftHeight(lift, Lift.GROUND_HEIGHT);
    }
}