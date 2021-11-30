package org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Sensors.BulkRead;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Input;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Output;

@Config
public class ElevatorOld implements Output, Input {

    //Elevator motor
    public DcMotorEx elevator;

    //Bulk Read
    private BulkRead bRead;
    private boolean useBRead;

    //Convert motor ticks to rotations (using Gobilda's given equation)
    private final double toRot = (((1+(46.0/17))) * (1+(46.0/17))) * 28;
    public static final double PULLEY_CIRCUMFERENCE = 2.8285; //inches
    public static final int STAGES = 2;

    public enum Level {
        GROUND(0),
        BARRIER(11.1),
        LOW(11.1),
        MIDDLE(17),
        HIGH(22),
        MAX(25);

        public final double height;

        Level(double height) {
            this.height = height;
        }
    }

    //PIDF Controller
    private PIDFController controller;
    private double output, lastOutput;
    public static PIDCoefficients pidCoeffs = new PIDCoefficients(0.01, 0, 0);

    //Position (in motor ticks) to run to
    private int currPosition, setPosition;

    //Power to use if not running PID
    private double elevatorPower, lastElevatorPower;

    //If using PID or not
    private boolean usePID;

    //If using manual override
    private boolean manualOverride;

    public ElevatorOld(HardwareMap hardwareMap, String elevatorName) {
        elevator = hardwareMap.get(DcMotorEx.class, elevatorName);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator.setDirection(DcMotorSimple.Direction.REVERSE);

        initPID();
        initVars();

        useBRead = false;
    }

    public ElevatorOld(HardwareMap hardwareMap, String elevatorName, BulkRead bRead) {
        elevator = hardwareMap.get(DcMotorEx.class, elevatorName);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator.setDirection(DcMotorSimple.Direction.REVERSE);

        initPID();
        initVars();

        this.bRead = bRead;
        useBRead = true;
    }

    public ElevatorOld(HardwareMap hardwareMap, String elevatorName, BulkRead bRead, boolean reset) {
        elevator = hardwareMap.get(DcMotorEx.class, elevatorName);
        if (reset)
            elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator.setDirection(DcMotorSimple.Direction.REVERSE);

        initPID();
        initVars();

        this.bRead = bRead;
        useBRead = true;
    }

    @Override
    public void updateInput() {
        //Get motor ticks
        if (useBRead) currPosition = -bRead.getMotorPos(elevator);
        else currPosition = elevator.getCurrentPosition();
    }

    @Override
    public void updateOutput() {
        //Use PID to go to position
        if (usePID && !manualOverride) {
            output = controller.update(currPosition); //Update PID

            //Stop the motor when at rest on the floor
            /*if (setPosition == 0 && currPosition < inchesToTicks(0.01)) {
                output = 0;
            }*/
            if (output != lastOutput) {
                elevator.setPower(output);

                lastOutput = output;
            }
        }

        //Use motor power to move elevator
        else {
            if (!manualOverride && elevatorPower < 0 && getHeight() < -0.05) elevatorPower = 0;

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

        if (usePID) lastElevatorPower = -2; //Impossible value so power != lastPower
        usePID = false;
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
    }

    /**
     * Gets the current position of the motor
     * @return Motor ticks
     */
    public int getMotorTicks() {
        return currPosition;
    }

    /**
     * Gets the current height of the elevator
     * @return Height in inches
     */
    public double getHeight() {
        return ticksToInches(currPosition);
    }

    /**
     * Converts motor ticks to the amount of inches of height
     * @param ticks Motor ticks
     * @return Amount of inches of height
     */
    private double ticksToInches(int ticks) {
        return (ticks / toRot) * PULLEY_CIRCUMFERENCE * STAGES;
    }

    /**
     * Converts amount of inches of height to motor ticks
     * @param inches Amount of inches of height
     * @return Motor ticks
     */
    private int inchesToTicks(double inches) {
        return (int) ((inches / STAGES / PULLEY_CIRCUMFERENCE) * toRot);
    }

    private void initPID() {
        controller = new PIDFController(pidCoeffs);
        controller.setOutputBounds(-1, 1);
    }

    private void initVars() {
        output = 0; lastOutput = 0;
        setPosition = 0;
        elevatorPower = 0; lastElevatorPower = 0;
        usePID = true;
        manualOverride = false;
    }

    @Override
    public void stopOutput() {
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        setPower(0);
        updateOutput();
    }
}
