package org.firstinspires.ftc.teamcode.Autonomous.WHSide;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.Auto_V2;
import org.firstinspires.ftc.teamcode.Autonomous.HubLevel;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Lift;
import static org.firstinspires.ftc.teamcode.Autonomous.HubLevel.*;

@Autonomous(name = "Red Warehouse")
@Disabled
public class RedWH extends Auto_V2 {

    @Override
    protected Alliance getAlliance() {
        return Alliance.RED;
    }

    @Override
    protected void auto() {
        HubLevel liftHeight = commands.detectBarcode(tseDetector);

        //Set lift to correct level according to the vision
        switch (liftHeight) {
            case LOW:
                commands.setLiftHeight(lift, Lift.LOW_HEIGHT);
            case MIDDLE:
                commands.setLiftHeight(lift, Lift.MIDDLE_HEIGHT);
            case HIGH:
                commands.setLiftHeight(lift, Lift.HIGH_HEIGHT);
        }

        //Drive to hub
        driveTo(24,0,0);

        //Deliver pre-loaded block
        commands.outtake(intake, 1500);

        //Move back a little so that the intake doesn't hit the hub
        moveTo(20, 0);

        //Put lift back down
        lift.setTargetHeight(Lift.GROUND_HEIGHT);

        //Realign with the wall and turn towards the warehouse
        rotateTo(Math.toRadians(270));
        moveTo(0,0);
        commands.setLiftHeight(lift, Lift.GROUND_HEIGHT);

        //Start intaking
        intake.setIntakePower(1);
 
        //Drive into the warehouse
        driveTo(0,-47, Math.toRadians(270));

        //Stop intake
        intake.setIntakePower(0);

        //Drive backwards to the hub
        driveTo(0,0, Math.toRadians(270));

        //Turn and move towards the hub
        driveTo(10,0,Math.toRadians(270));
        lift.setTargetHeight(Lift.HIGH_HEIGHT);
        driveTo(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), 0);

        //Set lift to the highest height
        commands.setLiftHeight(lift, Lift.HIGH_HEIGHT);

        //Move forward
        driveTo(16,0,0);

        //Outtake the game element
        commands.outtake(intake, 1500);

        //Drive a little back and turn
        driveTo(10,0, 0);

        //Put lift back down
        lift.setTargetHeight(Lift.GROUND_HEIGHT);

        //Realign with the wall and turn towards the warehouse
        driveTo(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), Math.toRadians(270));

        //Park in warehouse
        driveTo(0,0, Math.toRadians(270));
        commands.setLiftHeight(lift, Lift.GROUND_HEIGHT);
        driveTo(0,47, Math.toRadians(270));
    }
}