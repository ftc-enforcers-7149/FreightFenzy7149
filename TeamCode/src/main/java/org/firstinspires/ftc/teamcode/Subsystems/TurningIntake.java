package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

        intakePower = lastIntakePower;
        wristPos = lastWristPos;
    }

    public void setIntakePower(double intakePower) {
        this.intakePower = intakePower;
    }

    public void setWristPosLeft() {
        wristPos = 0;
    }

    public void setWristPosRight() {
        wristPos = 1;
    }

    public void setWristPosCenter() {
        wristPos = 0.615;
    }

    public void stop() {
        intake.setPower(0);
        wrist.setPosition(0.615);
    }
}
