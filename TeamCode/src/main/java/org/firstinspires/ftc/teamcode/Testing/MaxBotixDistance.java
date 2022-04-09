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

    CorrectedMB1220 maxbotixFL, maxbotixFR;
    private final int INITIAL_SMOOTHING = 9;

    public void init() {

        try {
            initializeAll();
        } catch (Exception e) {
            e.printStackTrace();
        }

        maxbotixFL = new CorrectedMB1220(hardwareMap, "distFL", INITIAL_SMOOTHING, MovingUltrasonicSensor.Facing.FRONT, drive.getLocalizer());
        maxbotixFR = new CorrectedMB1220(hardwareMap, "distFR", INITIAL_SMOOTHING, MovingUltrasonicSensor.Facing.FRONT, drive.getLocalizer());

        maxbotixFL.enableMoving();
        maxbotixFR.enableMoving();
    }

    public void loop() {

        getInput();

        if(gamepad1.a) {

            maxbotixFL.setQuartileSmoothing(false);
            maxbotixFR.setQuartileSmoothing(false);

        }

        if(gamepad1.b) {

            maxbotixFL.setQuartileSmoothing(false);
            maxbotixFR.setQuartileSmoothing(false);

            maxbotixFL.setSmoothingSize(1);
            maxbotixFR.setSmoothingSize(1);
        }

        if(gamepad1.x) {

            maxbotixFL.setQuartileSmoothing(true);
            maxbotixFR.setQuartileSmoothing(true);

            maxbotixFL.setSmoothingSize(INITIAL_SMOOTHING);
            maxbotixFR.setSmoothingSize(INITIAL_SMOOTHING);
        }

        telemetry.addData("Distance F: ", maxbotixFL.getDistance(DistanceUnit.INCH));
        telemetry.addData("Distance L: ", maxbotixFR.getDistance(DistanceUnit.INCH));
    }

    @Override
    protected void getInput() {
        maxbotixFL.updateInput();
        maxbotixFR.updateInput();
    }

    @Override
    protected void updateStateMachine() {

    }
}
