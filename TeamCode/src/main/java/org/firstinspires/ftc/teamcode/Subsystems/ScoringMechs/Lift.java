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
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Levels;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Output;

import static org.firstinspires.ftc.teamcode.Subsystems.Utils.Levels.GROUND;
import static org.firstinspires.ftc.teamcode.Subsystems.Utils.Levels.MAX;

@Config
public class Lift implements Output, Input {

    //Lift motor
    public DcMotorEx lift;

    //Bulk Read
    private BulkRead bRead;
    private boolean useBR;

    //Input
    private int currPosition;
    private int offset;

    //Convert motor ticks to rotations (using Gobilda's given equation)
    private final double ticksPerRot = ((((1+(46/17d))) * (1+(46/17d))) * 28);
    public static final double PULLEY_CIRCUMFERENCE = 2.8285; //inches
    public static final int STAGES = 2;
    private final double heightPerRot = PULLEY_CIRCUMFERENCE * STAGES;
    private final double ticksPerInch = ticksPerRot / heightPerRot;
    private final double inchesPerTick = heightPerRot / ticksPerRot;

    private double currHeight;

    //PIDF Controller
    private PIDFController controller;
    private double output, lastOutput;
    public static PIDCoefficients pidCoeffs = new PIDCoefficients(0.01, 0, 0);
    private int setPosition;

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

        useBR = false;
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
        useBR = true;
    }

    public Lift(HardwareMap hardwareMap, String liftName, BulkRead bRead, boolean reset) {
        lift = hardwareMap.get(DcMotorEx.class, liftName);
        if (reset) lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        initPID();
        initVars();

        this.bRead = bRead;
        useBR = true;
    }

    private void setCurrPosition(int position) {
        offset = position - currPosition;
        currPosition = position;
    }

    @Override
    public void updateInput() {
        //Get motor ticks
        if (useBR) currPosition = -bRead.getMotorPos(lift) + offset;
        else currPosition = lift.getCurrentPosition() + offset;

        currHeight = inchesPerTick * currPosition;
    }

    @Override
    public void updateOutput() {
        //Use PID to go to position
        if (usePID && !manualOverride) {
            output = controller.update(currPosition); //Update PID

            //Stop the motor when at rest on the floor
            if (setPosition == GROUND.height && currPosition < GROUND.height - 0.05) {
                output = 0;
            }
            if (output != lastOutput) {
                lift.setPower(output);

                lastOutput = output;
            }
        }

        //Use motor power to move lift
        else {
            if (!manualOverride && (
                    (liftPower < 0 && currHeight < -0.05) ||
                    (liftPower > 0 && currHeight > MAX.height + 0.05))) liftPower = 0;

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

        if (power != 0) {
            if (usePID) lastLiftPower = -2; //Impossible value so power != lastPower
            usePID = false;
        }
        else if (!manualOverride) { //Hold position
            setTargetHeight(getHeight());
        }
    }

    /**
     * Set a target height for the lift, starting the PID
     * @param inches Target height in inches
     */
    public void setTargetHeight(double inches) {
        //Keep bounds between 0 and MAX_HEIGHT
        setPosition = inchesToTicks(Math.min(Math.max(inches, GROUND.height), MAX.height));
        controller.setTargetPosition(setPosition);
        usePID = true;
    }

    /**
     * Set a target height for the elevator, starting the PID
     * @param level Target level
     */
    public void setTargetHeight(Levels level) {
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

    public double getMotorTicks() {
        return currPosition;
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
        usePID = true;
    }

    @Override
    public void stopOutput() {
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        setManualOverride(true);
        setPower(0);
        updateOutput();
    }
}
