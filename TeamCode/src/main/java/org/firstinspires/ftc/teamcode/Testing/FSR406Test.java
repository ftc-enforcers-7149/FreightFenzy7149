package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors.FSR406;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.LED.LED;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOp_Base;

@TeleOp(name="Force Test")
public class FSR406Test extends TeleOp_Base {

    private FSR406 force;
    private LED led;

    private double forceVal;

    public static final double LIGHT_RESISTANCE = 115000, MEDIUM_RESISTANCE = 5000, HEAVY_RESISTANCE = 10000, BOUND = 5000;

    @Override
    public void init() {

        force = new FSR406(hardwareMap, "force", 10000d, 15);
        force.setQuartileSmoothing(true);
        led = new LED(hardwareMap, "blinkin", Alliance.NONE);

    }

    @Override
    public void loop() {

        getInput();
        telemetry.addData("Force: ", forceVal);

        updateStateMachine();

    }

    @Override
    protected void getInput() {

        force.updateInput();
        forceVal = force.getWeight();

    }

    @Override
    protected void updateStateMachine() {

        if(forceVal >= LIGHT_RESISTANCE - BOUND && forceVal <= LIGHT_RESISTANCE + BOUND) led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        else if(forceVal >= MEDIUM_RESISTANCE - BOUND && forceVal <= MEDIUM_RESISTANCE + BOUND) led.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
        else if(forceVal >= HEAVY_RESISTANCE - BOUND && forceVal <= HEAVY_RESISTANCE + BOUND) led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        else led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GRAY);

    }
}
