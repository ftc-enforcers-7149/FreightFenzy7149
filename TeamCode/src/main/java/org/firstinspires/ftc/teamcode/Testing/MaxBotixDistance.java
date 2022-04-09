package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors.CorrectedMB1220;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors.MaxbotixMB1220;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors.MovingUltrasonicSensor;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOp_Base;

@TeleOp(name="MaxbotixTest")
public class MaxBotixDistance extends TeleOp_Base {

    CorrectedMB1220 maxbotixF, maxbotixL, maxbotixR;
    private final int INITIAL_SMOOTHING = 9;

    public void init() {

        try {
            initializeAll();
        } catch (Exception e) {
            e.printStackTrace();
        }

        maxbotixF = new CorrectedMB1220(hardwareMap, "distF", INITIAL_SMOOTHING, MovingUltrasonicSensor.Facing.FRONT, drive.getLocalizer());
        maxbotixL = new CorrectedMB1220(hardwareMap, "distL", INITIAL_SMOOTHING, MovingUltrasonicSensor.Facing.LEFT, drive.getLocalizer());
        maxbotixR = new CorrectedMB1220(hardwareMap, "distR", INITIAL_SMOOTHING, MovingUltrasonicSensor.Facing.RIGHT, drive.getLocalizer());

        maxbotixF.enableMoving();

    }

    public void loop() {

        getInput();

        if(gamepad1.a) {

            maxbotixF.setQuartileSmoothing(false);
            maxbotixL.setQuartileSmoothing(false);
            maxbotixR.setQuartileSmoothing(false);

        }

        if(gamepad1.b) {

            maxbotixF.setQuartileSmoothing(false);
            maxbotixL.setQuartileSmoothing(false);
            maxbotixR.setQuartileSmoothing(false);

            maxbotixF.setSmoothingSize(1);
            maxbotixL.setSmoothingSize(1);
            maxbotixR.setSmoothingSize(1);

        }

        if(gamepad1.x) {

            maxbotixF.setQuartileSmoothing(true);
            maxbotixL.setQuartileSmoothing(true);
            maxbotixR.setQuartileSmoothing(true);

            maxbotixF.setSmoothingSize(INITIAL_SMOOTHING);
            maxbotixL.setSmoothingSize(INITIAL_SMOOTHING);
            maxbotixR.setSmoothingSize(INITIAL_SMOOTHING);

        }

        telemetry.addData("Distance F: ", maxbotixF.getDistance(DistanceUnit.INCH));
        telemetry.addData("Distance L: ", maxbotixL.getDistance(DistanceUnit.INCH));
        telemetry.addData("Distance R: ", maxbotixR.getDistance(DistanceUnit.INCH));
    }

    @Override
    protected void getInput() {
        maxbotixF.updateInput();
        maxbotixL.updateInput();
        maxbotixR.updateInput();
    }

    @Override
    protected void updateStateMachine() {

    }
}
