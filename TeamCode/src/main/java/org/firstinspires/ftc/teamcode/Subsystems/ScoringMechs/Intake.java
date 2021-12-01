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

    private static final double minDistance = 1.5;
    private ValueTimer<Double> distance;
    private ValueTimer<Integer> redValue;
    private final boolean useSensor;

    //State machine logic
    private double intakePower,  lastIntakePower;

    public Intake(HardwareMap hardwaremap, String intakeServoName, String intakeColorSensorName) {
        intake = hardwaremap.crservo.get(intakeServoName);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeColorSensor = hardwaremap.get(RevColorSensorV3.class, intakeColorSensorName);

        distance = new ValueTimer<Double>(250) {
            @Override
            public Double readValue() {
                return intakeColorSensor.getDistance(DistanceUnit.INCH);
            }
        };
        redValue = new ValueTimer<Integer>(250) {
            @Override
            public Integer readValue() {
                return intakeColorSensor.red();
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
    public void updateInput() {
        if (useSensor) {
            distance.updateInput();
            redValue.updateInput();
        }
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
        return useSensor && (distance.getValue() < minDistance || redValue.getValue() > 100);
    }

    @Override
    public void startInput() {
        if (useSensor) {
            distance.startInput();
            redValue.startInput();
        }
    }

    @Override
    public void stopInput() {
        if (useSensor) {
            distance.stopInput();
            redValue.stopInput();
        }
    }

    @Override
    public void stopOutput() {
        setIntakePower(0);
        updateOutput();
    }
}
