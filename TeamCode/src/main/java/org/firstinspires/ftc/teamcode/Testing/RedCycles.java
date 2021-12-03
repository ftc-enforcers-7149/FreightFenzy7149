package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.AutoCommands;
import org.firstinspires.ftc.teamcode.Autonomous.Auto_V2;

@Autonomous(name = "Test Red Cycles")
//@Disabled
public class RedCycles extends Auto_V2 {

    @Override
    protected Alliance getAlliance() {
        return Alliance.RED;
    }

    @Override
    public void auto() {
        testToGap();
        //testThroughGap();
        //testToFreight();
        //testToHub();
        //testCycle();
    }

    private void testToGap() {
        driveTo(16, 0, 0);
        commands.driveToGap(lift, false);
        setMotorPowers(0, 0, 0, 0);
    }

    private void testThroughGap() {
        commands.driveThroughGap(drive, positioning, false);
        setMotorPowers(0, 0, 0, 0);
    }

    private void testToFreight() {
        try {
            commands.driveToFreightAndBack(drive, positioning, intake, false);
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
            commands.cycle(drive, positioning, lift, intake, false);
        }
        catch (AutoCommands.NoFreight ignored) {
            setMotorPowers(0, 0, 0, 0);
        }
    }
}