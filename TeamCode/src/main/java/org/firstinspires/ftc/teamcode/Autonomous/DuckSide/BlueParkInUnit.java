package org.firstinspires.ftc.teamcode.Autonomous.DuckSide;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.Auto_V3;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Elevator;

@Autonomous(name = "Blue Duck - Unit Park", preselectTeleOp = "Tele V3 Blue")
@Disabled
public class BlueParkInUnit extends Auto_V3 {

    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }

    @Override
    protected void auto() {
        Elevator.Level elevatorHeight = commands.detectBarcode(tseDetector);

        //Drive to the duckwheel
        driveTo(5, -4, 0);

        //Spin and stop duckwheel
        commands.spinDuck(spinner, 3000);

        //Drive to hub
        driveTo(34,26, Math.toRadians(60));

        //Set elevator to correct level according to the vision
        commands.setElevatorHeight(elevator, elevatorHeight);

        //Drive to hub and outtake
        driveTo(36,30, Math.toRadians(60));
        commands.outtake(intake);

        //Drive a little bit back and drop elevator
        driveTo(26,26, Math.toRadians(60));
        elevator.setTargetHeight(Elevator.Level.GROUND);

        //Align with the warehouse and park
        driveTo(26,20, Math.toRadians(90));
        driveTo(26,-12, Math.toRadians(90));

        //Lower elevator all the way down for TeleOp
        commands.setElevatorHeight(elevator, Elevator.Level.GROUND);
    }
}