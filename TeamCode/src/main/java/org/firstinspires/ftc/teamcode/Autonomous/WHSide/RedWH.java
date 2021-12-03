package org.firstinspires.ftc.teamcode.Autonomous.WHSide;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.AutoCommands;
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

        POS_ACC = 1;

        //Set lift to correct level according to the vision
        switch (liftHeight) {
            case LOW:
                lift.setTargetHeight(Lift.LOW_HEIGHT);
                break;
            case MIDDLE:
                lift.setTargetHeight(Lift.MIDDLE_HEIGHT);
                break;
            case HIGH:
                lift.setTargetHeight(Lift.HIGH_HEIGHT);
                break;
        }

        //Drive to hub and wait for lift
        driveTo(18, 0, 0);
        customWait(() -> (lift.getLiftHeight() < lift.getTargetHeight() - 0.5));

        //Deliver pre-loaded block
        driveTo(20, 0, 0);
        commands.outtake(intake);

        //Move back a little so that the intake doesn't hit the hub
        driveTo(16, 0, 0);

        //Put lift back down
        lift.setTargetHeight(Lift.GROUND_HEIGHT);

        //Cycles
        while (opModeIsActive() && getRuntime() < 25) {
            try {
                commands.cycle(drive, positioning, lift, intake, false);
            }
            catch (AutoCommands.NoFreight nF) {
                return;
            }
        }

        //Park
        commands.driveToGap(lift, false);
        commands.driveThroughGap(drive, positioning, false);

        driveTo(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY() - 10, 0);
    }
}