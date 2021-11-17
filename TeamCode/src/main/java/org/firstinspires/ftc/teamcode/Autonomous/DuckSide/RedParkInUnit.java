package org.firstinspires.ftc.teamcode.Autonomous.DuckSide;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.Auto_V2;
import org.firstinspires.ftc.teamcode.Autonomous.Autonomous_Base;
import org.firstinspires.ftc.teamcode.Autonomous.HubLevel;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.CarouselSpinner;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.TurningIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam.OpenCV;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam.TSEPipeline;
import org.opencv.core.RotatedRect;

@Autonomous(name = "Red Duck - Unit Park")
@Disabled
public class RedParkInUnit extends Auto_V2 {

    @Override
    protected Alliance getAlliance() {
        return Alliance.RED;
    }

    @Override
    protected void auto() {
        HubLevel liftHeight = commands.detectBarcode(tseDetector);

        //Drive to the duckwheel
        driveTo(5, 4, 0);

        //Spin and stop duckwheel
        commands.spinDuck(spinner, 4000);

        //Drive to hub
        driveTo(34,-26, Math.toRadians(315));

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

        //Drive to hub and outtake
        driveTo( 36,-30, Math.toRadians(315));
        commands.outtake(intake, 1500);

        //Drive a little bit back and drop lift
        driveTo(26,-26, Math.toRadians(315));
        lift.setTargetHeight(Lift.GROUND_HEIGHT);

        //Align with the warehouse and park
        driveTo(26,-20, Math.toRadians(270));
        driveTo(26,12, Math.toRadians(270));

        //Lower lift all the way down for TeleOp
        commands.setLiftHeight(lift, Lift.GROUND_HEIGHT);
    }
}