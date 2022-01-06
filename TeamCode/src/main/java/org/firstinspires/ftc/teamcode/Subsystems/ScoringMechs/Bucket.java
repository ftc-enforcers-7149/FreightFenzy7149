package org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Input;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Interp;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Output;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.ValueTimer;

public class Bucket implements Input, Output {

    //Hardware
    private DcMotorEx intake;
    private Servo bucket;

    //Sensors
    public RevColorSensorV3 color;
    private boolean useSensor;

    //Detection
    private static final double minDistance = 1.5;
    private ValueTimer<Boolean> intakeDetected;

    //Bucket positions
    public enum Position {
        IN(0), HOLD(0.65), OUT(1);

        public final double position;

        Position(double position) {
            this.position = position;
        }
    }

    //State machine logic
    public enum State {
        INTAKE, HOLD, OUTTAKE, IDLE
    }
    private State currState, lastState;

    private boolean holdToggle;

    private double intakePower,  lastIntakePower;
    private double position, lastPosition;

    private Interp interp;

    public Bucket(HardwareMap hardwareMap, String intakeName, String bucketName, String colorName) {
        intake = hardwareMap.get(DcMotorEx.class, intakeName);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bucket = hardwareMap.servo.get(bucketName);

        color = hardwareMap.get(RevColorSensorV3.class, colorName);

        intakeDetected = new ValueTimer<Boolean>(false, 250) {
            @Override
            public Boolean readValue() {
                return color.getDistance(DistanceUnit.INCH) < minDistance;
            }
        };

        initVars();
        idle();

        useSensor = true;
    }

    public Bucket(HardwareMap hardwareMap, String intakeName, String bucketName) {
        intake = hardwareMap.get(DcMotorEx.class, intakeName);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bucket = hardwareMap.servo.get(bucketName);

        initVars();
        idle();

        useSensor = false;
    }

    public void setIntakePower(double intakePower) {
        this.intakePower = intakePower;
    }

    public void setBucketPosition(double position) {
        interp = new Interp(this.position, position, 0, Interp.Method.INSTANT);
    }

    public void setBucketPosition(Position position) {
        setBucketPosition(position.position);
    }

    public void intake(boolean intake) {
        if (intake) {
            transitionState(State.INTAKE);
            holdToggle = false;
        }
        else
            holdToggle();
    }

    public void hold(boolean hold) {
        if (hold) transitionState(State.HOLD);
        else transitionState(State.IDLE);

        holdToggle = hold;
    }

    public void holdToggle() {
        hold(!holdToggle);
    }

    public void outtake(boolean outtake) {
        if (outtake) {
            transitionState(State.OUTTAKE);
            holdToggle = false;
        }
        else
            holdToggle();
    }

    public void idle() {
        transitionState(State.IDLE);
        holdToggle = false;
    }

    private void transitionState(State newState) {
        lastState = currState;
        currState = newState;
        if (currState != lastState) {
            switch (currState) {
                case INTAKE:
                    setBucketPosition(Position.IN);
                    intakePower = 0.8;
                    break;
                case OUTTAKE:
                    setBucketPosition(Position.OUT);
                    intakePower = 0;
                    break;
                case HOLD:
                    if (lastState == State.INTAKE)
                        intakePower = -0.3;
                    else
                        intakePower = 0;
                    setBucketPosition(Position.HOLD);
                    break;
                case IDLE:
                    setBucketPosition(Position.IN);
                    intakePower = 0;
                    break;
            }
        }
    }

    private void handleState() {
        switch (currState) {
            case INTAKE:
                if (useSensor && intakeDetected.getValue()) intakePower = 0;
                //if (intakePower == 0) transitionState(State.IDLE);
                break;
            case OUTTAKE:
                if (useSensor && !intakeDetected.getValue()) intakePower = 0;
                //if (intakePower == 0) transitionState(State.IDLE);
                break;
            case HOLD:
            case IDLE:
                //intakePower = 0;
                //if (intakePower > 0) transitionState(State.INTAKE);
                //else if (intakePower < 0) transitionState(State.OUTTAKE);
                break;
        }
    }

    @Override
    public void startOutput() {
        bucket.setPosition(position);
        intake.setPower(intakePower);
    }

    @Override
    public void updateOutput() {
        handleState();

        //Bucket movement
        position = interp.getValue();

        //if (position != lastPosition)
            bucket.setPosition(position);

        //Intake movement
        //if (intakePower != lastIntakePower)
            intake.setPower(intakePower);
    }

    @Override
    public void stopOutput() {
        setIntakePower(0);
        setBucketPosition(0);
        updateOutput();
    }

    @Override
    public void startInput() {
        if (useSensor) intakeDetected.startInput();
    }

    @Override
    public void updateInput() {
        if (useSensor) intakeDetected.updateInput();
    }

    @Override
    public void stopInput() {
        if (useSensor) intakeDetected.stopInput();
    }

    public boolean getFreightDetected() {
        return useSensor && intakeDetected.getValue();
    }

    private void initVars() {
        intakePower = 0; lastIntakePower = 0;
        position = Position.IN.position; lastPosition = Position.IN.position;
        interp = new Interp(position, position, 0, Interp.Method.INSTANT);
        currState = State.IDLE; lastState = State.IDLE;

        holdToggle = false;
    }
}
