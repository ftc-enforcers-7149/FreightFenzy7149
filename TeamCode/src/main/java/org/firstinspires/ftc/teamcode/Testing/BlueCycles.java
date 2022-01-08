/*
package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.AutoCommands;
import org.firstinspires.ftc.teamcode.Autonomous.Auto_V2;

@Autonomous(name = "Test Blue Cycles")
@Disabled
public class BlueCycles extends Auto_V2 {

    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }

    @Override
    public void auto() {
        POS_ACC = 2;

        testToGap();
        testThroughGap();
        //testToFreight();
        //testToHub();
        //testCycle();

        //Cycles
        */
/*while (opModeIsActive() && getRuntime() < 25) {
            try {
                commands.cycle(drive, positioning, lift, intake, false);
            }
            catch (AutoCommands.NoFreight nF) {
                return;
            }
        }*//*

    }

    private void testToGap() {
        driveTo(16, 0, 0);
        commands.driveToGap(drive, positioning, lift);
        drive.stopOutput();
    }

    private void testThroughGap() {
        commands.driveThroughGap(drive, positioning);
        setMotorPowers(0, 0, 0, 0);
    }

    private void testToFreight() {
        try {
            commands.driveToFreightAndBack(drive, positioning, intake);
        }
        catch (AutoCommands.NoFreight ignored) {}
        finally {
            setMotorPowers(0, 0, 0, 0);
        }
    }

    private void testToHub() {
        commands.driveToHub(lift, intake);
    }

    private void testCycle() {
        try {
            commands.cycle(drive, positioning, lift, intake);
        }
        catch (AutoCommands.NoFreight ignored) {
            setMotorPowers(0, 0, 0, 0);
        }
    }
}*/
