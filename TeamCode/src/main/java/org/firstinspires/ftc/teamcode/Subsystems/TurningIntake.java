package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TurningIntake {

    public Servo wrist;
    public CRServo intake;
    private double intakePower,lastIntakePower;
    private double wristPos, lastWristPos;

    public TurningIntake(HardwareMap hardwaremap, String intakeServoName, String wristServoName) {
        intake = hardwaremap.crservo.get(intakeServoName);
        wrist = hardwaremap.servo.get(wristServoName);

        wrist.setDirection(Servo.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        wrist.setPosition(0.615);
        intakePower = 0;
        lastIntakePower = 0;
        wristPos = 0.615;
        lastWristPos = 0.615;
    }

    public void update() {

        if (intakePower != lastIntakePower) {
            intake.setPower(intakePower);
        }
        if (wristPos != lastWristPos) {
            wrist.setPosition(wristPos);
        }

        lastIntakePower = intakePower;
        lastWristPos = wristPos;
    }

    public void setIntakePower(double intakePower) {
        this.intakePower = intakePower;
    }

    public void moveWristLeft() {
        wristPos -= 0.05;
        if (wristPos < 0) wristPos = 0;
    }

    public void moveWristRight() {
        wristPos += 0.05;
        if (wristPos > 1) wristPos = 1;
    }

    public void setWristCenter() {
        wristPos = 0.615;
    }

    public void stop() {
        setWristCenter();
        setIntakePower(0);
        update();
    }

    public double getIntakePower() {
        return intakePower;
    }

    public double getLastIntakePower() {
        return lastIntakePower;
    }

    public void setLastIntakePower(double lastIntakePower) {
        this.lastIntakePower = lastIntakePower;
    }

    public double getWristPos() {
        return wristPos;
    }

    public void setWristPos(double wristPos) {
        this.wristPos = wristPos;
    }

    public double getLastWristPos() {
        return lastWristPos;
    }

    public void setLastWristPos(double lastWristPos) {
        this.lastWristPos = lastWristPos;
    }
}
