package org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Subsytem;

public class Intake implements Subsytem {

    //Intake servos
    public CRServo intake;

    //State machine logic
    private double intakePower,lastIntakePower;

    public Intake(HardwareMap hardwaremap, String intakeServoName, String wristServoName) {
        intake = hardwaremap.crservo.get(intakeServoName);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        intakePower = 0;
        lastIntakePower = 0;
    }

    public Intake(HardwareMap hardwaremap, String intakeServoName) {
        intake = hardwaremap.crservo.get(intakeServoName);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        intakePower = 0;
        lastIntakePower = 0;
    }

    @Override
    public void update() {
        if (intakePower != lastIntakePower) {
            intake.setPower(intakePower);
        }

        lastIntakePower = intakePower;
    }

    /**
     * Set intake power
     * @param intakePower Servo power [-1,1]
     */
    public void setIntakePower(double intakePower) {
        this.intakePower = intakePower;
    }

    @Override
    public void stop() {
        setIntakePower(0);
        update();
    }
}
