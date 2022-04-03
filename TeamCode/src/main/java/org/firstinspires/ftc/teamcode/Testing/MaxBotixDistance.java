package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors.MaxbotixMB1220;

@TeleOp(name="MaxbotixTest")
public class MaxBotixDistance extends OpMode {

    MaxbotixMB1220 maxbotixF, maxbotixL, maxbotixR;
    private final int INITIAL_SMOOTHING = 9;

    public void init() {
        maxbotixF = new MaxbotixMB1220(hardwareMap, "distF", MaxbotixMB1220.VOLTAGE.THREE, INITIAL_SMOOTHING);
        maxbotixL = new MaxbotixMB1220(hardwareMap, "distL", MaxbotixMB1220.VOLTAGE.THREE, INITIAL_SMOOTHING);
        maxbotixR = new MaxbotixMB1220(hardwareMap, "distR", MaxbotixMB1220.VOLTAGE.THREE, INITIAL_SMOOTHING);

        maxbotixF.setQuartileSmoothing(true);
        maxbotixL.setQuartileSmoothing(true);
        maxbotixR.setQuartileSmoothing(true);

    }

    public void loop() {
        maxbotixF.updateInput();
        maxbotixL.updateInput();
        maxbotixR.updateInput();

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
}
