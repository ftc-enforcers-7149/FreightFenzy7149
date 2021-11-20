package org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Subsytem;
import org.firstinspires.ftc.teamcode.Subsystems.ValueTimer;

public class Intake implements Subsytem {

    //Intake servos
    public CRServo intake;

    //Sensors
    public RevColorSensorV3 intakeColorSensor;

    protected double intakeLength = 2;
    ValueTimer<Double> colorDist;

    //State machine logic
    private double intakePower,lastIntakePower;

    public Intake(HardwareMap hardwaremap, String intakeServoName, String intakeColorSensorName) {
        intakeColorSensor = hardwaremap.get(RevColorSensorV3.class, intakeColorSensorName);
        intake = hardwaremap.crservo.get(intakeServoName);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        colorDist = new ValueTimer<Double>() {
            @Override
            public Double readValue() {
                return intakeColorSensor.getDistance(DistanceUnit.INCH);
            }
        };

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
        colorDist.update();

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
        return colorDist.getValue() < intakeLength;
    }

    @Override
    public void stop() {
        colorDist.pause();
        setIntakePower(0);
        update();
    }
}
