package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors.MaxbotixMB1220;

@TeleOp(name="MaxbotixTest")
public class MaxBotixDistance extends OpMode {

    MaxbotixMB1220 maxbotixF, maxbotixL, maxbotixR;

    public void init() {
        maxbotixF = new MaxbotixMB1220(hardwareMap, "distF", MaxbotixMB1220.VOLTAGE.THREE, 9);
        maxbotixL = new MaxbotixMB1220(hardwareMap, "distL", MaxbotixMB1220.VOLTAGE.THREE, 15);
        maxbotixR = new MaxbotixMB1220(hardwareMap, "distR", MaxbotixMB1220.VOLTAGE.THREE, 17);

        maxbotixF.setQuartileSmoothing(true);
        maxbotixL.setQuartileSmoothing(true);
        maxbotixR.setQuartileSmoothing(true);

    }

    public void loop() {
        maxbotixF.updateInput();
        maxbotixL.updateInput();
        maxbotixR.updateInput();
        telemetry.addData("Distance F: ", maxbotixF.getDistance(DistanceUnit.INCH));
        telemetry.addData("Distance L: ", maxbotixL.getDistance(DistanceUnit.INCH));
        telemetry.addData("Distance R: ", maxbotixR.getDistance(DistanceUnit.INCH));
    }
}
