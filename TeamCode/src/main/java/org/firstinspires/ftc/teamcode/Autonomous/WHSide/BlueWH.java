package org.firstinspires.ftc.teamcode.Autonomous.WHSide;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.Auto_V2;
import org.firstinspires.ftc.teamcode.Autonomous.HubLevel;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Lift;

@Autonomous(name = "Blue Warehouse")
@Disabled
public class BlueWH extends Auto_V2 {

    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }

    @Override
    protected void auto() {
        HubLevel liftHeight = commands.detectBarcode(tseDetector);

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

        //Drive to hub
        driveTo(16, 0, 0);

        //Deliver pre-loaded block
        commands.outtake(intake, 1500);

        //Move back a little so that the intake doesn't hit the hub
        driveTo(10, 0, 0);

        //Put lift back down
        lift.setTargetHeight(Lift.GROUND_HEIGHT);

        //Realign with the wall and turn towards the warehouse
        driveTo(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), Math.toRadians(90));
        driveTo(0, 0, Math.toRadians(90));
        commands.setLiftHeight(lift, Lift.GROUND_HEIGHT);

        //Start intaking
        intake.setIntakePower(1);

        //Drive into the warehouse
        driveTo(0, 47, Math.toRadians(90));

        //Stop intake
        intake.setIntakePower(0);

        //Drive backwards to the hub
        driveTo(0, 0, Math.toRadians(90));

        //Turn and move towards the hub
        driveTo(10, 0, Math.toRadians(90));
        lift.setTargetHeight(Lift.HIGH_HEIGHT);
        driveTo(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), 0);

        //Set lift to the highest height
        commands.setLiftHeight(lift, Lift.HIGH_HEIGHT);

        //Move forward
        driveTo(16, 0, 0);

        //Outtake the game element
        commands.outtake(intake, 1500);

        //Drive a little back and turn
        driveTo(10, 0, 0);

        //Put lift back down
        lift.setTargetHeight(Lift.GROUND_HEIGHT);

        //Realign with the wall and turn towards the warehouse
        driveTo(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), Math.toRadians(90));

        //Park in warehouse
        driveTo(0, 0, Math.toRadians(90));
        commands.setLiftHeight(lift, Lift.GROUND_HEIGHT);
        driveTo(0, -47, Math.toRadians(90));
    }
}
