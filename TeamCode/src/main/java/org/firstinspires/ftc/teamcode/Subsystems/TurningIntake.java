package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TurningIntake {

    //Intake servos
    public Servo wrist;
    public CRServo intake;

    //State machine logic
    private double intakePower,lastIntakePower;
    private double wristPos, lastWristPos;

    //Scan turning
    private double turnSpeed;
    private long lastTime;

    public TurningIntake(HardwareMap hardwaremap, String intakeServoName, String wristServoName) {
        intake = hardwaremap.crservo.get(intakeServoName);
        wrist = hardwaremap.servo.get(wristServoName);

        wrist.setDirection(Servo.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        wrist.setPosition(0.45);
        intakePower = 0;
        lastIntakePower = 0;
        wristPos = 0.45;
        lastWristPos = 0.45;
        turnSpeed = 0;
        lastTime = System.currentTimeMillis();
    }

    public TurningIntake(HardwareMap hardwaremap, String intakeServoName, String wristServoName,
                         boolean moveOnInit) {
        intake = hardwaremap.crservo.get(intakeServoName);
        wrist = hardwaremap.servo.get(wristServoName);

        wrist.setDirection(Servo.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        if (moveOnInit) wrist.setPosition(0.45);
        intakePower = 0;
        lastIntakePower = 0;
        wristPos = 0.45;
        lastWristPos = moveOnInit ? 0.45 : -1;
        turnSpeed = 0;
        lastTime = System.currentTimeMillis();
    }

    public void update() {
        if (intakePower != lastIntakePower) {
            intake.setPower(intakePower);
        }

        //Move wrist at [turnSpeed]/sec
        wristPos += turnSpeed * (System.currentTimeMillis() - lastTime) / 1000;

        if (wristPos < 0) wristPos = 0;
        if (wristPos > 1) wristPos = 1;

        if (wristPos != lastWristPos) {
            wrist.setPosition(wristPos);//fixInput(wristPos));
        }

        //Set turnSpeed back to 0 after each loop so
        //the wrist stops moving if moveWrist isn't called
        turnSpeed = 0;

        lastIntakePower = intakePower;
        lastWristPos = wristPos;
        lastTime = System.currentTimeMillis();
    }

    /**
     * Set intake power
     * @param intakePower Servo power [-1,1]
     */
    public void setIntakePower(double intakePower) {
        this.intakePower = intakePower;
    }

    /**
     * Scan wrist to the left
     */
    public void moveWristLeft() {
        turnSpeed = -1;
    }

    /**
     * Scan wrist to the right
     */
    public void moveWristRight() {
        turnSpeed = 1;
    }

    /**
     * Turn wrist all the way to the left
     */
    public void setWristLeft() {
        wristPos = 0; turnSpeed = 0;
    }

    /**
     * Turn wrist all the way to the right
     */
    public void setWristRight() {
        wristPos = 1; turnSpeed = 0;
    }

    /**
     * Turn wrist to the center
     */
    public void setWristCenter() {
        wristPos = 0.45; turnSpeed = 0;
    }

    /**
     * Move wrist to a custom position
     * @param pos Servo position [0,1]
     */
    public void setWristPos(double pos) {
        wristPos = pos; turnSpeed = 0;
    }

    /**
     * Scan wrist at a custom speed
     * @param speed Scanning speed [-inf,inf]
     */
    public void setWristSpeed(double speed) {
        turnSpeed = speed;
    }

    /**
     * Fix linear input [0,1] with 0.5 as center to the curved output [0,1] with 0.45 as center
     * @param input Linear input [0,1]
     * @return Curved output [0,1]
     */
    private double fixInput(double input) {
        return (-0.2) * (input*input) + (0.8) * (input);
    }

    public void stop() {
        //setWristCenter();
        setIntakePower(0);
        update();
    }
}
