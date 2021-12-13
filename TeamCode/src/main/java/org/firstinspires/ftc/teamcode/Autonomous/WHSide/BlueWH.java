package org.firstinspires.ftc.teamcode.Autonomous.WHSide;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.Auto_V3;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Elevator;

@Autonomous(name = "Blue Warehouse", preselectTeleOp = "Tele V3 Blue")
@Disabled
public class BlueWH extends Auto_V3 {

    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }

    @Override
    protected void auto() {
        Elevator.Level elevatorHeight = commands.detectBarcode(tseDetector);

        //Set elevator to correct level according to the vision
        commands.setElevatorHeight(elevator, elevatorHeight);

        //Drive to hub
        driveTo(16, 0, 0);

        //Deliver pre-loaded block
        commands.outtake(intake);

        //Move back a little so that the intake doesn't hit the hub
        driveTo(10, 0, 0);

        //Put elevator back down
        elevator.setTargetHeight(Elevator.Level.GROUND);

        //Realign with the wall and turn towards the warehouse
        driveTo(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), Math.toRadians(90));
        driveTo(0, 0, Math.toRadians(90));
        commands.setElevatorHeight(elevator, Elevator.Level.GROUND);

        //Start intaking
        intake.intake();

        //Drive into the warehouse
        driveTo(0, 47, Math.toRadians(90));

        //Drive backwards to the hub
        driveTo(0, 0, Math.toRadians(90));

        //Turn and move towards the hub
        driveTo(10, 0, Math.toRadians(90));
        elevator.setTargetHeight(Elevator.Level.HIGH);
        driveTo(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), 0);

        //Set elevator to the highest height
        commands.setElevatorHeight(elevator, Elevator.Level.HIGH);

        //Move forward
        driveTo(16, 0, 0);

        //Outtake the game element
        commands.outtake(intake);

        //Drive a little back and turn
        driveTo(10, 0, 0);

        //Put elevator back down
        elevator.setTargetHeight(Elevator.Level.GROUND);

        //Realign with the wall and turn towards the warehouse
        driveTo(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), Math.toRadians(90));

        //Park in warehouse
        driveTo(0, 0, Math.toRadians(90));
        commands.setElevatorHeight(elevator, Elevator.Level.GROUND);
        driveTo(0, -47, Math.toRadians(90));
    }
}
