package org.firstinspires.ftc.teamcode.Autonomous.DuckSide;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.Auto_V2_5;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.MotorIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Levels;

import static org.firstinspires.ftc.teamcode.GlobalData.*;

@Autonomous(name = "Blue Duck")
@Disabled
public class BlueDuck extends Auto_V2_5 {

    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }

    @Override
    protected void auto() {
        drive.setPoseEstimate(new Vector2d(0, 15));

        Levels liftHeight = commands.detectBarcode(tseDetector);

        SLOW_DIST = 25;

        //Set lift to correct level according to the vision
        Lift.pidCoeffs = new PIDCoefficients(0.004, 0, 0.0001);
        lift.initPID();
        lift.setTargetHeight(liftHeight);   //TODO: Four Bar

        //Drive to hub
        SLOW_DIST = 10;
        driveTo(27,24, Math.toRadians(50));
        SLOW_DIST = 25;

        //Drive to hub and outtake
        driveTo(31,29, Math.toRadians(50));
        commands.outtake(intake, lift);

        Lift.pidCoeffs = new PIDCoefficients(0.006, 0, 0.00015);
        lift.initPID();
        lift.setTargetHeight(Levels.GROUND);    //TODO: Four Bar

        //Drive to the duckwheel
        POS_ACC = 3;
        SPEED_MULT = 0.7;
        driveTo(-7, -16, Math.toRadians(90), 1500);
        SPEED_MULT = 1;
        POS_ACC = 1;

        //Spin and stop duckwheel
        commands.spinDuck(spinner);

        drive.setPoseEstimate(new Pose2d(0, -14, Math.toRadians(90)));

        waitForTime(750);

        //Try to intake duck
        intake.setIntakePower(0.5);
        driveTo(7, -12, Math.toRadians(160));
        driveTo(7, -2, Math.toRadians(160));
        driveTo(10, 9, Math.toRadians(90));
        intake.setIntakePower(0);
        intake.setLatch(MotorIntake.LatchPosition.DUCK_CLOSED);

        lift.setTargetHeight(Levels.HIGH);  //TODO: Four Bar

        //Drive to hub
        driveTo(25,21, Math.toRadians(60));

        //Drive to hub and outtake
        driveTo( 29,26, Math.toRadians(60));
        commands.outtake(intake, lift);

        //Back away from hub
        driveTo(19,23, Math.toRadians(60));
        lift.setTargetHeight(Levels.GROUND);    //TODO: Four Bar

        //Park in storage unit
        driveTo(28.5, -25, Math.toRadians(0), 1500);

        //Lower lift all the way down for TeleOp
        commands.setLiftHeight(lift, Levels.GROUND);
    }
}