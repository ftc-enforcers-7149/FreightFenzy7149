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
public class Lift implements Output, Input {

    //Lift motor
    public DcMotorEx lift;

    //Bulk Read
    private BulkRead bRead;
    private boolean useBRead;

    //Convert motor ticks to rotations (using Gobilda's given equation)
    private final double toRot = (((1+(46.0/17))) * (1+(46.0/17))) * 28;
    public static final double PULLEY_CIRCUMFERENCE = 2.8285; //inches
    public static final int STAGES = 2;

    public static double GROUND_HEIGHT = 0;
    public static double BARRIER_HEIGHT = 5;
    public static double LOW_HEIGHT = 5.5;
    public static double MIDDLE_HEIGHT = 11;
    public static double HIGH_HEIGHT = 15.5;
    public static double MAX_HEIGHT = 22;

    //PIDF Controller
    private PIDFController controller;
    private double output, lastOutput;
    public static PIDCoefficients pidCoeffs = new PIDCoefficients(0.01, 0, 0);

    //Position (in motor ticks) to run to
    private int currPosition, setPosition;

    //Power to use if not running PID
    private double liftPower, lastLiftPower;

    //If using PID or not
    private boolean usePID;

    //If using manual override
    private boolean manualOverride;

    public Lift(HardwareMap hardwareMap, String liftName) {
        lift = hardwareMap.get(DcMotorEx.class, liftName);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        initPID();
        initVars();

        useBRead = false;
    }

    public Lift(HardwareMap hardwareMap, String liftName, BulkRead bRead) {
        lift = hardwareMap.get(DcMotorEx.class, liftName);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        initPID();
        initVars();

        this.bRead = bRead;
        useBRead = true;
    }

    public Lift(HardwareMap hardwareMap, String liftName, BulkRead bRead, boolean reset) {
        lift = hardwareMap.get(DcMotorEx.class, liftName);
        if (reset)
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        initPID();
        initVars();

        this.bRead = bRead;
        useBRead = true;
    }

    @Override
    public void updateInput() {
        //Get motor ticks
        if (useBRead) currPosition = -bRead.getMotorPos(lift);
        else currPosition = lift.getCurrentPosition();
    }

    @Override
    public void updateOutput() {
        //Use PID to go to position
        if (usePID && !manualOverride) {
            output = controller.update(currPosition); //Update PID

            //Stop the motor when at rest on the floor
            if (setPosition == Lift.GROUND_HEIGHT && currPosition < Lift.GROUND_HEIGHT-liftInchesToTicks(0.01)) {
                output = 0;
            }
            if (output != lastOutput) {
                lift.setPower(output);

                lastOutput = output;
            }
        }

        //Use motor power to move lift
        else {
            if (!manualOverride && liftPower < 0 && getLiftHeight() < -0.05) liftPower = 0;

            if (liftPower != lastLiftPower) {
                lift.setPower(liftPower);

                lastLiftPower = liftPower;
            }
        }
    }

    /**
     * Set power to the lift motor, stopping the PID
     * @param power Motor power
     */
    public void setPower(double power) {
        liftPower = power;

        if (usePID) lastLiftPower = -2;
        usePID = false;
    }

    /**
     * Set a target height for the lift, starting the PID
     * @param inches Target height in inches
     */
    public void setTargetHeight(double inches) {
        //Keep bounds between 0 and MAX_HEIGHT
        setPosition = liftInchesToTicks(Math.min(Math.max(inches, 0), MAX_HEIGHT));
        controller.setTargetPosition(setPosition);
        usePID = true;
    }

    public double getTargetHeight() {
        return Math.abs(ticksToLiftInches(setPosition));
    }

    /**
     * Manual override disables all features that use the encoder
     * This consists of the automatic PID control and over turn protection
     * @param override Whether or not to override
     */
    public void setManualOverride(boolean override) {
        manualOverride = override;
        if (override) currPosition = 0;
        else {
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /**
     * Gets the current position of the motor
     * @return Motor ticks
     */
    public int getMotorTicks() {
        return currPosition;
    }

    /**
     * Gets the current height of the lift
     * @return Height in inches
     */
    public double getLiftHeight() {
        return ticksToLiftInches(currPosition);
    }

    /**
     * Converts motor ticks to the amount of inches of lift
     * @param ticks Motor ticks
     * @return Amount of inches of lift
     */
    private double ticksToLiftInches(int ticks) {
        return (ticks / toRot) * PULLEY_CIRCUMFERENCE * STAGES;
    }

    /**
     * Converts amount of inches of lift to motor ticks
     * @param inches Amount of inches of lift
     * @return Motor ticks
     */
    private int liftInchesToTicks(double inches) {
        return (int) ((inches / STAGES / PULLEY_CIRCUMFERENCE) * toRot);
    }

    private void initPID() {
        controller = new PIDFController(pidCoeffs);
        controller.setOutputBounds(-1, 1);
    }

    private void initVars() {
        output = 0; lastOutput = 0;
        setPosition = 0;
        liftPower = 0; lastLiftPower = 0;
        usePID = true;
        manualOverride = false;
    }

    @Override
    public void stopOutput() {
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        setPower(0);
        updateOutput();
    }
}
