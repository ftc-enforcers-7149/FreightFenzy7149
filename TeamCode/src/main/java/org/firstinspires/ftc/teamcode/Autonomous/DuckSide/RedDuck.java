package org.firstinspires.ftc.teamcode.Autonomous.DuckSide;

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
        HubLevel liftHeight = commands.detectBarcode(tseDetector);

        POS_ACC = 1;
        SLOW_DIST = 15;

        //Drive to the duckwheel
        driveTo(5, 4, 0);

        //Spin and stop duckwheel
        spinner.setLeftPower(0.75);
        waitForTime(4000);
        spinner.setLeftPower(0);

        //Drive to hub
        driveTo(34,-26, Math.toRadians(307));

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
        driveTo( 36,-31, Math.toRadians(300));
        commands.outtake(intake, 1500);

        H_ACC = Math.toRadians(3);

        //Drive a little bit back and drop lift
        driveTo(32,-26, Math.toRadians(300));
        lift.setTargetHeight(Lift.GROUND_HEIGHT);

        while (getRuntime() < 22) {
            updateInputs();
            updateOutputs();
        }
        lift.setTargetHeight(Lift.BARRIER_HEIGHT);

        //Align with the warehouse and park
        driveTo(30,-33, Math.toRadians(100));
        commands.setLiftHeight(lift, Lift.BARRIER_HEIGHT);

        SLOW_DIST = 20;
        driveTo(40,-128, Math.toRadians(100));

        //Lower lift all the way down for TeleOp
        commands.setLiftHeight(lift, Lift.GROUND_HEIGHT);
    }
}