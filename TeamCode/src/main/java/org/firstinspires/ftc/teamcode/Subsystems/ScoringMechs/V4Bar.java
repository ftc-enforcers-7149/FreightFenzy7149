package org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Utils.Input;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Output;

public class V4Bar implements Output {

    //Left and right servos
    public Servo left, right;

    public V4Bar(HardwareMap hardwareMap, String leftName, String rightName) {
        left = hardwareMap.servo.get(leftName);
        right = hardwareMap.servo.get(rightName);
    }

    @Override
    public void updateOutput() {

    }
}
