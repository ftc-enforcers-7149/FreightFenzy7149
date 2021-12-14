package org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Input;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Output;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.ValueTimer;

import java.util.ArrayList;

public class Intake implements Output, Input {

    //Intake servos
    public CRServo intake;

    //Sensors
    public RevColorSensorV3 intakeColor, outtakeColor;
    private boolean useSensor;

    private static final double minDistance = 1.5;
    private static final double blockRedColor = 100;
    private ValueTimer<Boolean> intakeDetected, intakeBlock, outtakeDetected;

    //State machine logic
    private double intakePower,  lastIntakePower;

    private enum IntakeState {
        IDLE, INTAKE, OUTTAKE_UP, OUTTAKE_DOWN, TO_OUTTAKE, TO_INTAKE;

        public IntakeState forward() {
            switch (this) {
                case IDLE:
                    return INTAKE;
                case INTAKE:
                case TO_INTAKE:
                    return TO_OUTTAKE;
                case TO_OUTTAKE:
                    return OUTTAKE_UP;
                case OUTTAKE_UP:
                case OUTTAKE_DOWN:
                default:
                    return IDLE;
            }
        }

        public IntakeState backward() {
            switch (this) {
                case IDLE:
                case TO_INTAKE:
                    return OUTTAKE_DOWN;
                case TO_OUTTAKE:
                    return TO_INTAKE;
                case OUTTAKE_UP:
                case OUTTAKE_DOWN:
                case INTAKE:
                default:
                    return IDLE;
            }
        }
    }
    private IntakeState state = IntakeState.IDLE;
    private ArrayList<IntakeState> nextStates;

    private long startTime;

    public Intake(HardwareMap hardwaremap, String intakeServoName, String intakeColorName, String outtakeColorName) {
        intake = hardwaremap.crservo.get(intakeServoName);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeColor = hardwaremap.get(RevColorSensorV3.class, intakeColorName);
        outtakeColor = hardwaremap.get(RevColorSensorV3.class, outtakeColorName);

        intakeDetected = new ValueTimer<Boolean>(false, 250) {
            @Override
            public Boolean readValue() {
                return intakeColor.getDistance(DistanceUnit.INCH) < minDistance;
            }
        };
        intakeBlock = new ValueTimer<Boolean>(false, 250) {
            @Override
            public Boolean readValue() {
                return intakeColor.red() > blockRedColor;
            }
        };
        outtakeDetected = new ValueTimer<Boolean>(false, 250) {
            @Override
            public Boolean readValue() {
                return outtakeColor.getDistance(DistanceUnit.INCH) < minDistance;
            }
        };

        nextStates = new ArrayList<IntakeState>();

        intakePower = 0;
        lastIntakePower = 0;
        useSensor = true;
    }

    public Intake(HardwareMap hardwaremap, String intakeServoName) {
        intake = hardwaremap.crservo.get(intakeServoName);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        nextStates = new ArrayList<IntakeState>();

        intakePower = 0;
        lastIntakePower = 0;
        useSensor = false;
    }

    @Override
    public void updateInput() {
        if (useSensor) {
            intakeDetected.updateInput();
            intakeBlock.updateInput();
            outtakeDetected.updateInput();
        }
    }

    @Override
    public void updateOutput() {
        if (useSensor) {
            switch (state) {
                case IDLE:
                    nextState();
                    break;
                case INTAKE:
                    intakeDetected.setDelayTime(50);
                    if (!intakeDetected.getValue()) intakePower = 1;
                    else {
                        intakeDetected.setDelayTime(250);
                        nextState();
                    }
                    break;
                case OUTTAKE_DOWN:
                    if (intakeDetected.getValue() && System.currentTimeMillis() < startTime + 1500)
                        intakePower = -1;
                    else nextState();
                    break;
                case OUTTAKE_UP:
                    if (outtakeDetected.getValue() && System.currentTimeMillis() < startTime + 1500)
                        intakePower = 1;
                    else nextState();
                    break;
                case TO_OUTTAKE:
                    outtakeDetected.setDelayTime(50);
                    if (!outtakeDetected.getValue() && System.currentTimeMillis() < startTime + 1500)
                        intakePower = 1;
                    else {
                        outtakeDetected.setDelayTime(250);
                        nextState();
                    }
                    break;
                case TO_INTAKE:
                    intakeDetected.setDelayTime(50);
                    if (!intakeDetected.getValue() && System.currentTimeMillis() < startTime + 1500)
                        intakePower = -1;
                    else {
                        intakeDetected.setDelayTime(250);
                        nextState();
                    }
                    break;
            }
        }
        else {
            switch (state) {
                case IDLE:
                    intakePower = 0;
                    break;
                case INTAKE:
                    intakePower = 1;
                    break;
                case OUTTAKE_DOWN:
                    intakePower = -1;
                    break;
            }
        }

        if (intakePower != lastIntakePower) {
            intake.setPower(intakePower);
        }

        lastIntakePower = intakePower;
    }

    public void intake() {
        nextStates.clear();
        if (useSensor) {
            nextStates.add(IntakeState.INTAKE);
        }
        else {
            state = IntakeState.INTAKE;
        }
    }

    public void outtakeUp() {
        nextStates.clear();
        if (useSensor) {
            nextStates.add(IntakeState.TO_OUTTAKE);
            nextStates.add(IntakeState.OUTTAKE_UP);
        }
        else {
            state = IntakeState.INTAKE;
        }
    }

    public void outtakeDown() {
        nextStates.clear();
        if (useSensor) {
            if (outtakeDetected.getValue()) nextStates.add(IntakeState.TO_INTAKE);
            nextStates.add(IntakeState.OUTTAKE_DOWN);
        }
        else {
            state = IntakeState.OUTTAKE_DOWN;
        }
    }

    public boolean isBusy() {
        return state != IntakeState.IDLE;
    }

    public void forward() {
        if (useSensor)
            nextStates.add(state.forward());
        else {
            nextStates.clear();
            switch (state) {
                case IDLE:
                    state = IntakeState.INTAKE;
                    break;
                case INTAKE:
                case OUTTAKE_DOWN:
                    state = IntakeState.IDLE;
                    break;
            }
        }
    }

    public void backward() {
        if (useSensor)
            nextStates.add(state.backward());
        else {
            nextStates.clear();
            switch (state) {
                case IDLE:
                    state = IntakeState.OUTTAKE_DOWN;
                    break;
                case INTAKE:
                case OUTTAKE_DOWN:
                    state = IntakeState.IDLE;
                    break;
            }
        }
    }

    private void nextState() {
        if (nextStates.size() > 0) {
            state = nextStates.remove(0);
            startTime = System.currentTimeMillis();
        }
        else {
            intakePower = 0;
            state = IntakeState.IDLE;
        }
    }

    @Override
    public void startInput() {
        if (useSensor) {
            intakeDetected.startInput();
            //intakeBlock.startInput();
            outtakeDetected.startInput();
        }
    }

    @Override
    public void stopInput() {
        if (useSensor) {
            intakeDetected.stopInput();
            intakeBlock.stopInput();
            outtakeDetected.stopInput();
        }
    }

    @Override
    public void stopOutput() {
        state = IntakeState.IDLE;
        nextStates.clear();
        updateOutput();
    }
}
