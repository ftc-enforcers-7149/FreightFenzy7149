package org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Input;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Output;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.ValueTimer;

public class MotorIntake implements Input, Output {

    //Intake hardware
    public DcMotorEx intake;
    public Servo paddle, latch;

    //Sensors
    public RevColorSensorV3 intakeColorSensor;

    private static final double minDistance = 2;
    private ValueTimer<Double> distance;
    private final boolean useSensor;

    //State machine logic
    private double intakePower = 0,  lastIntakePower = 0;

    public enum PaddlePosition {
        BACK(1),
        OUT(0.3),
        IDLE(BACK.pos);

        public final double pos;

        PaddlePosition(double pos) {
            this.pos = pos;
        }
    }
    public enum LatchPosition {
        OPEN(1),
        CLOSED(0.4),
        IDLE(OPEN.pos);

        public final double pos;

        LatchPosition(double pos) {
            this.pos = pos;
        }
    }

    private PaddlePosition currPaddle = PaddlePosition.BACK, lastPaddle = PaddlePosition.IDLE;
    private LatchPosition currLatch = LatchPosition.OPEN, lastLatch = LatchPosition.IDLE;

    public MotorIntake(HardwareMap hardwaremap,
                       String motorName, String paddleName, String latchName,
                       String intakeColorSensorName) {
        intake = hardwaremap.get(DcMotorEx.class, motorName);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        paddle = hardwaremap.servo.get(paddleName);
        latch = hardwaremap.servo.get(latchName);

        intakeColorSensor = hardwaremap.get(RevColorSensorV3.class, intakeColorSensorName);

        distance = new ValueTimer<Double>(0.0, 250) {
            @Override
            public Double readValue() {
                return intakeColorSensor.getDistance(DistanceUnit.INCH);
            }
        };

        useSensor = true;
    }

    public MotorIntake(HardwareMap hardwaremap,
                  String motorName, String paddleName, String latchName) {
        intake = hardwaremap.get(DcMotorEx.class, motorName);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        paddle = hardwaremap.servo.get(paddleName);
        latch = hardwaremap.servo.get(latchName);

        useSensor = false;
    }

    @Override
    public void updateInput() {
        if (useSensor) {
            distance.updateInput();
        }
    }

    @Override
    public void updateOutput() {
        if (intakePower != lastIntakePower) {
            intake.setPower(intakePower);
            lastIntakePower = intakePower;
        }

        if (currPaddle != lastPaddle) {
            paddle.setPosition(currPaddle.pos);
            lastPaddle = currPaddle;
        }

        if (currLatch != lastLatch) {
            latch.setPosition(currLatch.pos);
            lastLatch = currLatch;
        }
    }

    /**
     * Set intake power
     * @param intakePower Servo power [-1,1]
     */
    public void setIntakePower(double intakePower) {
        this.intakePower = intakePower;
    }

    public void setPaddle(PaddlePosition pos) {
        currPaddle = pos;
    }
    public void setLatch(LatchPosition pos) {
        currLatch = pos;
    }

    public boolean getFreightInIntake () {
        return useSensor && (distance.getValue() < minDistance);
    }

    public void startScanningIntake() {
        distance.setDelayTime(0);
    }

    public void stopScanningIntake() {
        distance.setDelayTime(250);
    }

    @Override
    public void startInput() {
        if (useSensor)
            distance.startInput();
    }

    @Override
    public void stopInput() {
        if (useSensor)
            distance.stopInput();
    }

    @Override
    public void stopOutput() {
        setIntakePower(0);
        setPaddle(PaddlePosition.BACK);
        setLatch(LatchPosition.OPEN);
        updateOutput();
    }
}
