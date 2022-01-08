/*
package org.firstinspires.ftc.teamcode.Autonomous.WHSide;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.AutoCommands;
import org.firstinspires.ftc.teamcode.Autonomous.Auto_V2;
import org.firstinspires.ftc.teamcode.Autonomous.HubLevel;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Lift;

import java.util.function.Supplier;

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
        lift.setTargetHeight(Lift.GROUND_HEIGHT);

        //Move back a little so that the intake doesn't hit the hub
        driveTo(10, 0, Math.toRadians(270));

        //Drive to wall
        driveTo(() -> (drive.getPoseEstimate().getX() - positioning.getRightDistance()),
                drive.getPoseEstimate()::getY, drive.getPoseEstimate()::getHeading);

        waitForTime(3000); //Debug and safety

        drive.setPoseEstimate(new Pose2d(0, drive.getPoseEstimate().getY(), Math.toRadians(270)));

        //Drive through gap
        driveTo(0, -40, Math.toRadians(270));

        // STOP HERE TO PARK NEAR THE GAP

        //Park along back wall
        driveTo(24, -45, Math.toRadians(225));
        driveTo(24, -64, Math.toRadians(180));

        //Against wall
        driveTo(drive.getPoseEstimate()::getX,
                () -> (drive.getPoseEstimate().getY() - positioning.getLeftDistance()),
                drive.getPoseEstimate()::getHeading);

        drive.setPoseEstimate(new Pose2d(
                drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), Math.toRadians(180))
        );
    }
}*/
