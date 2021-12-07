package org.firstinspires.ftc.teamcode.Autonomous.WHSide;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.Auto_V3;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.ElevatorOld;

@Autonomous(name = "Red Warehouse")
@Disabled
public class RedWH extends Auto_V3 {

    @Override
    protected Alliance getAlliance() {
        return Alliance.RED;
    }

    @Override
    protected void auto() {
        ElevatorOld.Level elevatorHeight = commands.detectBarcode(tseDetector);

        //Set elevator to correct level according to the vision
        //commands.setElevatorHeight(elevator, elevatorHeight);

        //Drive to hub
        driveTo(24,0,0);

        //Deliver pre-loaded block
        commands.outtake(intake);

        //Move back a little so that the intake doesn't hit the hub
        moveTo(20, 0);

        //Put elevator back down
        //elevator.setTargetHeight(ElevatorOld.Level.GROUND);

        //Realign with the wall and turn towards the warehouse
        rotateTo(Math.toRadians(270));
        moveTo(0,0);
        //commands.setElevatorHeight(elevator, ElevatorOld.Level.GROUND);

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
        //elevator.setTargetHeight(ElevatorOld.Level.HIGH);
        driveTo(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), 0);

        //Set elevator to the highest height
        //commands.setElevatorHeight(elevator, ElevatorOld.Level.HIGH);

        //Move forward
        driveTo(16,0,0);

        //Outtake the game element
        commands.outtake(intake);

        //Drive a little back and turn
        driveTo(10,0, 0);

        //Put elevator back down
        //elevator.setTargetHeight(ElevatorOld.Level.GROUND);

        //Realign with the wall and turn towards the warehouse
        driveTo(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), Math.toRadians(270));

        //Park in warehouse
        driveTo(0,0, Math.toRadians(270));
        //commands.setElevatorHeight(elevator, ElevatorOld.Level.GROUND);
        driveTo(0,47, Math.toRadians(270));
    }
}