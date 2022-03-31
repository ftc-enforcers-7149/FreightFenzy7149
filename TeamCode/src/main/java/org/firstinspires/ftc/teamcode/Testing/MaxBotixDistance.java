package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Sensors.MaxbotixMB1220;

@TeleOp(name="MaxbotixTest")
public class MaxBotixDistance extends OpMode {

    MaxbotixMB1220 maxbotix;

    public void init() {
        maxbotix = new MaxbotixMB1220(hardwareMap, "Distance", MaxbotixMB1220.VOLTAGE.THREE, 9);
    }

    public void loop() {
        maxbotix.updateInput();
        telemetry.addData("Distance: ", maxbotix.getDistance() * 0.393701d);
    }
}
