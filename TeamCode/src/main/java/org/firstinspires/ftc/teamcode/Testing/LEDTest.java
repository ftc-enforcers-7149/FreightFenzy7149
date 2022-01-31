package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.GlobalData;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.LED.LED;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOp_Base;

@TeleOp(name="Blinkin Test")
//@Disabled
public class LEDTest extends TeleOp_Base {

    private LED led;

    private BlinkinPattern pattern;

    private boolean next, prev, lastNext, lastPrev;
    private boolean switchAlliance, lastSwitchAlliance;

    @Override
    public void init() {
        initializeSources();
        initializeBulkRead();
        initializeGyro();
        initializeVars();

        led = new LED(hardwareMap, "blinkin", Alliance.BLUE);

        GlobalData.ALLIANCE = Alliance.BLUE;
        pattern = BlinkinPattern.BLACK;

        addOutput(led);
    }

    @Override
    public void start() {
        startInputs();
        startOutputs();
    }

    @Override
    public void loop() {
        updateInputs();
        getInput();

        if (next && !lastNext) {
            pattern = pattern.next();
        }
        else if (prev && !lastPrev) {
            pattern = pattern.previous();
        }

        if (switchAlliance && !lastSwitchAlliance) {
            if (GlobalData.ALLIANCE == Alliance.BLUE) GlobalData.ALLIANCE = Alliance.RED;
            else GlobalData.ALLIANCE = Alliance.BLUE;
        }

        led.setPattern(pattern);

        updateOutputs();
        updateStateMachine();
    }

    @Override
    public void stop() {
        stopInputs();
        stopOutputs();
    }

    @Override
    protected void getInput() {
        next = gamepad1.dpad_right;
        prev = gamepad1.dpad_left;
        switchAlliance = gamepad1.x;
    }

    @Override
    protected void updateStateMachine() {
        lastNext = next;
        lastPrev = prev;
        lastSwitchAlliance = switchAlliance;
    }
}
