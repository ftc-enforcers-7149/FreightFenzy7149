package org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs;

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

    private boolean wheelInterferes;

    public TurningIntake(HardwareMap hardwaremap, String intakeServoName, String wristServoName) {
        intake = hardwaremap.crservo.get(intakeServoName);
        wrist = hardwaremap.servo.get(wristServoName);

        wrist.setDirection(Servo.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        wrist.setPosition(0.32);
        intakePower = 0;
        lastIntakePower = 0;
        wristPos = 0.32;
        lastWristPos = 0.32;
        turnSpeed = 0;
        lastTime = System.currentTimeMillis();

        wheelInterferes = true;
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
        wristPos = 0.32;
        lastWristPos = moveOnInit ? 0.32 : -1;
        turnSpeed = 0;
        lastTime = System.currentTimeMillis();

        wheelInterferes = true;
    }

    public void update() {
        if (intakePower != lastIntakePower) {
            intake.setPower(intakePower);
        }

        //Move wrist at [turnSpeed]/sec
        wristPos += turnSpeed * (System.currentTimeMillis() - lastTime) / 1000;

        if (!wheelInterferes) {
            if (wristPos < 0) wristPos = 0;
            if (wristPos > 1) wristPos = 1;
        }
        else {
            if (wristPos < 0.1) wristPos = 0.1;
            if (wristPos > 0.55) wristPos = 0.55;
        }

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
        if (wheelInterferes) wristPos = 0;
        else  wristPos = 0.1;
        turnSpeed = 0;
    }

    /**
     * Turn wrist all the way to the right
     */
    public void setWristRight() {
        if (wheelInterferes) wristPos = 1;
        else  wristPos = 0.55;
        turnSpeed = 0;
    }

    /**
     * Turn wrist to the center
     */
    public void setWristCenter() {
        wristPos = 0.32; turnSpeed = 0;
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
     * Change the left and right limits of the servo so it doesn't hit the wheels
     * when they're in the way
     * @param interferes Whether or not the wheels will interfere with the servo limits
     */
    public void setWheelInterferes(boolean interferes) {
        wheelInterferes = interferes;
    }

    /**
     * Fix linear input [0,1] with 0.5 as center to the curved output [0,1] with 0.375 as center
     * @param input Linear input [0,1]
     * @return Curved output [0,1]
     */
    private double fixInput(double input) {
        return (0.5) * (input*input) + (0.5) * (input);
    }

    public void stop() {
        setIntakePower(0);
        update();
    }
}
