package org.firstinspires.ftc.teamcode.Autonomous.DuckSide;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.Auto_V3;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.ElevatorOld;

@Autonomous(name = "Blue Duck")
//@Disabled
public class BlueDuck extends Auto_V3 {

    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }

    @Override
    protected void auto() {
        ElevatorOld.Level elevatorHeight = commands.detectBarcode(tseDetector);

        POS_ACC = 1;
        SLOW_DIST = 15;

        //Drive to the duckwheel
        driveTo(5, -4, 0);

        //Spin and stop duckwheel
        commands.spinDuck(spinner, 4000);

        //Drive to hub
        driveTo(34,26, Math.toRadians(60));

        //Set elevator to correct level according to the vision
        commands.setElevatorHeight(elevator, elevatorHeight);

        //Drive to hub and outtake
        driveTo(36,30, Math.toRadians(60));
        commands.outtake(intake, 1500);

        H_ACC = Math.toRadians(6);

        //Drive a little bit back and drop elevator
        driveTo(32,26, Math.toRadians(60));
        elevator.setTargetHeight(ElevatorOld.Level.GROUND);

        while (getRuntime() < 22) {
            updateInputs();
            updateOutputs();
        }
        elevator.setTargetHeight(ElevatorOld.Level.GROUND);

        //Align with the warehouse and park
        driveTo(26,33, Math.toRadians(270));
        commands.setElevatorHeight(elevator, ElevatorOld.Level.BARRIER);

        SLOW_DIST = 20;
        driveTo(21,128, Math.toRadians(270));

        //Lower elevator all the way down for TeleOp
        commands.setElevatorHeight(elevator, ElevatorOld.Level.GROUND);
    }
}