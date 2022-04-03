package org.firstinspires.ftc.teamcode.Autonomous.WHSide;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.Auto_V2_5;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.ArmController;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Levels;

import static org.firstinspires.ftc.teamcode.GlobalData.*;

@Autonomous(name = "Blue WH Cycles NEW NEW")
public class BlueCyclesNEWNEW extends Auto_V2_5 {

    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }

    @Override
    protected void auto() {
        distCorrect.startRunning();

        //Initial setup
        drive.setPoseEstimate(new Pose2d(6.75, 78.25, 0));
        //Levels levels = commands.detectBarcode(tseDetector);
        Levels levels = Levels.HIGH;

        //Move arm up
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        armController.setScorePos(ArmController.ScoringPosition.UP);

        //Start driving towards hub as arm gets out of intake
        SPEED_MULT = 0.5;
        driveTo(10, 78.25, 0);


        distCorrect.stopRunning();
    }
}
