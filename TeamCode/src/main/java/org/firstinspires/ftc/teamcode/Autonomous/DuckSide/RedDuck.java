package org.firstinspires.ftc.teamcode.Autonomous.DuckSide;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.Auto_V2_5;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.MotorIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Levels;

import static org.firstinspires.ftc.teamcode.GlobalData.*;

@Autonomous(name = "Red Duck")
//@Disabled
public class RedDuck extends Auto_V2_5 {

    @Override
    protected Alliance getAlliance() {
        return Alliance.RED;
    }

    @Override
    protected void auto() {
        drive.setPoseEstimate(new Vector2d(0, -15));

        Levels liftHeight = commands.detectBarcode(tseDetector);

        SLOW_DIST = 25;

        //Set lift to correct level according to the vision
        Lift.pidCoeffs = new PIDCoefficients(0.004, 0, 0.0001);
        lift.initPID();
        lift.setTargetHeight(liftHeight);

        //Drive to hub
        SLOW_DIST = 10;
        driveTo(25,-23, Math.toRadians(310));
        SLOW_DIST = 25;

        //Drive to hub and outtake
        driveTo( 29,-28, Math.toRadians(310));
        commands.outtake(intake, lift);

        Lift.pidCoeffs = new PIDCoefficients(0.006, 0, 0.00015);
        lift.initPID();
        lift.setTargetHeight(Levels.GROUND);

        //Drive to the duckwheel
        POS_ACC = 3;
        SPEED_MULT = 0.7;
        driveTo(-5, 14, Math.toRadians(270), 1500);
        SPEED_MULT = 1;
        POS_ACC = 1;

        //Spin and stop duckwheel
        commands.spinDuck(spinner);

        drive.setPoseEstimate(new Pose2d(3, 15, Math.toRadians(270)));

        waitForTime(750);

        //Try to intake duck
        intake.setIntakePower(0.5);
        driveTo(7, 12, Math.toRadians(210));
        driveTo(7, 2, Math.toRadians(210));
        driveTo(10, -9, Math.toRadians(270));
        intake.setIntakePower(0);
        intake.setLatch(MotorIntake.LatchPosition.DUCK_CLOSED);

        lift.setTargetHeight(Levels.HIGH);

        //Drive to hub
        driveTo(23,-20, Math.toRadians(320));

        //Drive to hub and outtake
        driveTo( 27,-25, Math.toRadians(320));
        //kpop
        commands.outtake(intake, lift);

        //Back away from hub
        driveTo(19,-23, Math.toRadians(320));
        lift.setTargetHeight(Levels.GROUND);

        //Park in storage unit
        driveTo(27,25, Math.toRadians(0), 1500);

        //Lower lift all the way down for TeleOp
        commands.setLiftHeight(lift, Levels.GROUND);
    }
}