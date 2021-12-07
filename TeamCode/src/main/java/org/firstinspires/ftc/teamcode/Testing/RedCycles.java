package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.AutoCommands;
import org.firstinspires.ftc.teamcode.Autonomous.Auto_V2;

@Autonomous(name = "Test Red Cycles")
@Disabled
public class RedCycles extends Auto_V2 {

    @Override
    protected Alliance getAlliance() {
        return Alliance.RED;
    }

    @Override
    public void auto() {
        POS_ACC = 2;

        //Pre-Setup
        driveTo(16, 0, 0);

        //Move to gap
        driveTo(0, -18, Math.toRadians(270));

        //Drive through gap
        positioning.startPositioning();
        driveTo(() -> (drive.getPoseEstimate().getX() - positioning.getRightDistance()),
                () -> {
                    if (positioning.getLineDetected()) return drive.getPoseEstimate().getY();
                    else return drive.getPoseEstimate().getY() - 20;
                }, () -> Math.toRadians(270));
        positioning.stopPositioning();
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
}