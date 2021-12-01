package org.firstinspires.ftc.teamcode.Autonomous.WHSide;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.Auto_V2;
import org.firstinspires.ftc.teamcode.Autonomous.HubLevel;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Lift;

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
                break;
            case MIDDLE:
                commands.setLiftHeight(lift, Lift.MIDDLE_HEIGHT);
                break;
            case HIGH:
                commands.setLiftHeight(lift, Lift.HIGH_HEIGHT);
                break;
        }

        //Drive to hub
        driveTo(20, 0, 0);

        //Deliver pre-loaded block
        commands.outtake(intake, 1000);

        //Move back a little so that the intake doesn't hit the hub
        driveTo(10, 0, 0);

        //Put lift back down
        lift.setTargetHeight(Lift.GROUND_HEIGHT);

        //Cycles
        for (int i = 0; i < 3; i++) {
            commands.cycle(drive, positioning, lift, intake, false);
        }

        //Park
        commands.driveToGap(lift, false);
        commands.driveThroughGap(drive, positioning, false);

        driveTo(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY() - 10, 0);
    }
}