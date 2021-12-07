package org.firstinspires.ftc.teamcode.Autonomous.DuckSide;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.Auto_V3;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.ElevatorOld;

@Autonomous(name = "Red Duck - Unit Park")
@Disabled
public class RedParkInUnit extends Auto_V3 {

    @Override
    protected Alliance getAlliance() {
        return Alliance.RED;
    }

    @Override
    protected void auto() {
        ElevatorOld.Level elevatorHeight = commands.detectBarcode(tseDetector);

        //Drive to the duckwheel
        driveTo(5, 4, 0);

        //Spin and stop duckwheel
        commands.spinDuck(spinner, 4000);

        //Drive to hub
        driveTo(34,-26, Math.toRadians(315));

        //Set elevator to correct level according to the vision
        //commands.setElevatorHeight(elevator, elevatorHeight);

        //Drive to hub and outtake
        driveTo( 36,-30, Math.toRadians(315));
        commands.outtake(intake);

        //Drive a little bit back and drop elevator
        driveTo(26,-26, Math.toRadians(315));
        //elevator.setTargetHeight(ElevatorOld.Level.GROUND);

        //Align with the warehouse and park
        driveTo(26,-20, Math.toRadians(270));
        driveTo(26,12, Math.toRadians(270));

        //Lower elevator all the way down for TeleOp
        //commands.setElevatorHeight(elevator, ElevatorOld.Level.GROUND);
    }
}