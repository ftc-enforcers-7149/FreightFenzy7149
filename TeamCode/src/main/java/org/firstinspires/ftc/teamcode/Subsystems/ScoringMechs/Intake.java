package org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Input;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Output;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.ValueTimer;

public class Intake implements Output, Input {

    //Intake servos
    public CRServo intake;

    //Sensors
    public RevColorSensorV3 intakeColorSensor;

    private static final double minDistance = 2;
    private ValueTimer<Double> colorDist;
    private boolean useSensor;

    //State machine logic
    private double intakePower,  lastIntakePower;

    public Intake(HardwareMap hardwaremap, String intakeServoName, String intakeColorSensorName) {
        intake = hardwaremap.crservo.get(intakeServoName);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeColorSensor = hardwaremap.get(RevColorSensorV3.class, intakeColorSensorName);

        colorDist = new ValueTimer<Double>() {
            @Override
            public Double readValue() {
                return intakeColorSensor.getDistance(DistanceUnit.INCH);
            }
        };

        intakePower = 0;
        lastIntakePower = 0;
        useSensor = true;
    }

    public Intake(HardwareMap hardwaremap, String intakeServoName) {
        intake = hardwaremap.crservo.get(intakeServoName);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        intakePower = 0;
        lastIntakePower = 0;
        useSensor = false;
    }

    @Override
    public void start() {
        if (useSensor) colorDist.start();
    }

    @Override
    public void updateInput() {
        if (useSensor) colorDist.updateInput();
    }

    @Override
    public void updateOutput() {
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

    public boolean getFreightInIntake () {
        return useSensor && colorDist.getValue() < minDistance;
    }

    @Override
    public void startInput() {
        colorDist.start();
    }

    @Override
    public void stopInput() {
        colorDist.stop();
    }

    @Override
    public void stop() {
        if (useSensor) colorDist.stop();
        setIntakePower(0);
        updateOutput();
    }
}
