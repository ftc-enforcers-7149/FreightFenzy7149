package org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Sensors.BulkRead;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Input;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Output;

public class Elevator implements Input, Output {

    //Elevator motor
    public DcMotorEx elevator;

    //Bulk Read
    public BulkRead bRead;
    private final boolean useBR;

    //Input
    private int currPosition = 0;
    private int offset = 0;

    //Convert motor ticks to rotations (using Gobilda's given equation)
    private final double ticksPerRot = ((((1+(46/11d))) * (1+(46/11d))) * 28);
    private final double heightPerRot = 3.14961; //10 Teeth * Chain pitch = 80mm (in inches)
    private final double ticksPerInch = ticksPerRot / heightPerRot;
    private final double inchesPerTick = heightPerRot / ticksPerRot;

    private double currHeight = 0;

    public enum Level {
        GROUND(0),
        BARRIER(5),
        LOW(5),
        MIDDLE(10),
        HIGH(15),
        MAX(16.5);

        public final double height;

        Level(double height) {
            this.height = height;
        }
    }

    //PIDF Controller
    private PIDFController controller;
    private double output, lastOutput;
    public static PIDCoefficients pidCoeffs = new PIDCoefficients(0.01, 0, 0);
    private int setPosition = 0;

    //Power to use if not running PID
    private double elevatorPower, lastElevatorPower;

    //If using PID or not
    private boolean usePID;

    //If using manual override
    private boolean manualOverride;

    public Elevator(HardwareMap hardwareMap, String elevatorName, BulkRead bRead) {
        elevator = hardwareMap.get(DcMotorEx.class, elevatorName);
        elevator.setDirection(DcMotorSimple.Direction.REVERSE);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.bRead = bRead;

        initPID();
        initVars();

        useBR = true;
    }

    public Elevator(HardwareMap hardwareMap, String elevatorName) {
        elevator = hardwareMap.get(DcMotorEx.class, elevatorName);
        elevator.setDirection(DcMotorSimple.Direction.REVERSE);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        initPID();
        initVars();

        useBR = false;
    }

    private void setCurrPosition(int position) {
        offset = position - currPosition;
        currPosition = position;
    }

    @Override
    public void updateInput() {
        //Get motor ticks
        if (useBR) currPosition = -bRead.getMotorPos(elevator) + offset;
        else currPosition = elevator.getCurrentPosition() + offset;

        currHeight = inchesPerTick * currPosition;
    }

    @Override
    public void updateOutput() {
        //Use PID to go to position
        if (usePID && !manualOverride) {
            output = controller.update(currPosition); //Update PID

            //Stop the motor when at rest on the floor
            if (setPosition == 0 && currHeight < -0.05) {
                output = 0;
            }
            if (output != lastOutput) {
                elevator.setPower(output);

                lastOutput = output;
            }
        }

        //Use motor power to move elevator
        else {
            if (!manualOverride &&
                    ((elevatorPower < 0 && currHeight < -0.05)
                    || (elevatorPower > 0 && currHeight > Level.MAX.height + 0.05))) elevatorPower = 0;

            if (elevatorPower != lastElevatorPower) {
                elevator.setPower(elevatorPower);

                lastElevatorPower = elevatorPower;
            }
        }
    }

    /**
     * Set power to the elevator motor, stopping the PID
     * @param power Motor power
     */
    public void setPower(double power) {
        elevatorPower = power;

        if (power != 0) {
            if (usePID) lastElevatorPower = -2; //Impossible value so power != lastPower
            usePID = false;
        }
        else if (!manualOverride) { //Hold position
            setTargetHeight(getHeight());
        }
    }

    /**
     * Set a target height for the elevator, starting the PID
     * @param inches Target height in inches
     */
    public void setTargetHeight(double inches) {
        //Keep bounds between 0 and MAX_HEIGHT
        setPosition = inchesToTicks(Math.min(Math.max(inches, 0), Level.MAX.height));
        controller.setTargetPosition(setPosition);
        usePID = true;
    }

    /**
     * Set a target height for the elevator, starting the PID
     * @param level Target level
     */
    public void setTargetHeight(Level level) {
        setTargetHeight(level.height);
    }

    /**
     * Manual override disables all features that use the encoder
     * This consists of the automatic PID control and over turn protection
     * @param override Whether or not to override
     */
    public void setManualOverride(boolean override) {
        manualOverride = override;
        if (override) setCurrPosition(0);
    }

    public double getHeight() {
        return !manualOverride ? currHeight : 0;
    }

    /**
     * Converts amount of inches of height to motor ticks
     * @param inches Amount of inches of height
     * @return Motor ticks
     */
    private int inchesToTicks(double inches) {
        return (int) (inches * ticksPerInch);
    }

    private void initPID() {
        controller = new PIDFController(pidCoeffs);
        controller.setOutputBounds(-1, 1);
    }

    private void initVars() {
        output = 0; lastOutput = 0;
        setPosition = 0;
    }

    @Override
    public void stopOutput() {
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        setManualOverride(true);
        setPower(0);
        updateOutput();
    }
}
