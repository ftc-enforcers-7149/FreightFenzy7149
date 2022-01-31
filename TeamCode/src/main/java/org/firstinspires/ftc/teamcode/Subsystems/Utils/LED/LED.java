package org.firstinspires.ftc.teamcode.Subsystems.Utils.LED;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Output;

public class LED implements Output {
    public RevBlinkinLedDriver blinkin;
    private BlinkinPattern pattern, lastPattern;

    private Alliance alliance;

    public LED(HardwareMap hardwareMap, String ledName, Alliance alliance) {
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, ledName);

        pattern = BlinkinPattern.HEARTBEAT_WHITE;
        lastPattern = BlinkinPattern.BLACK;

        this.alliance = alliance;
    }

    /**
     * sets LED pattern
     * @param pattern
     */
    public void setPattern(BlinkinPattern pattern) {
        this.pattern = pattern;
    }

    @Override
    public void updateOutput() {
        if (pattern != lastPattern) {
            blinkin.setPattern(pattern);
            lastPattern = pattern;
        }
    }

    @Override
    public void stopOutput() {
        setPattern(BlinkinPattern.BLACK);
        updateOutput();
    }
}
