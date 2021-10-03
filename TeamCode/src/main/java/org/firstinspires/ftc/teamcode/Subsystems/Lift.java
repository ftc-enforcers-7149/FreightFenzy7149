package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Lift {

    //Lift motor
    public DcMotorEx lift;

    //Bulk Read
    private BulkRead bRead;
    private boolean useBRead;

    //Convert motor ticks to rotations (using Gobilda's given equation)
    private final double toRot = (((1+(46.0/17))) * (1+(46.0/17))) * 28;
    public static final double PULLEY_CIRCUMFERENCE = 3.424; //inches
    public static final int STAGES = 2;
    public static final double MAX_HEIGHT = 13;

    //PIDF Controller
    private PIDFController controller;
    private double output, lastOutput;
    public static final PIDCoefficients pidCoeffs = new PIDCoefficients(0, 0, 0);

    //Position (in motor ticks) to run to
    private int currPosition, setPosition;

    //Power to use if not running PID
    private double liftPower, lastLiftPower;

    //If using PID or not
    private boolean usePID;

    /**
     * Initialize the lift (without Bulk Read)
     * @param hardwareMap The OpMode's hardware map
     * @param liftName The name of the lift motor in the configuration
     */
    public Lift(HardwareMap hardwareMap, String liftName) {
        lift = hardwareMap.get(DcMotorEx.class, liftName);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        initPID();
        initVars();

        useBRead = false;
    }

    /**
     * Initialize the lift (with Bulk Read)
     * @param hardwareMap The OpMode's hardware map
     * @param liftName The name of the lift motor in the configuration
     * @param bRead A Bulk Read object for the correct hub
     */
    public Lift(HardwareMap hardwareMap, String liftName, BulkRead bRead) {
        lift = hardwareMap.get(DcMotorEx.class, liftName);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        initPID();
        initVars();

        this.bRead = bRead;
        useBRead = true;
    }

    /**
     * Set power to the lift motor, stopping the PID
     * @param power Motor power
     */
    public void setPower(double power) {
        liftPower = power;
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

    /**
     * Updates the PID and lift power
     * When using PID, goes to a target position
     * When using motor power, does not go to a target
     */
    public void update() {
        //Get motor ticks
        if (useBRead) currPosition = Math.abs(bRead.getMotorPos(lift));
        else currPosition = Math.abs(lift.getCurrentPosition());

        //Use PID to go to position
        if (usePID) {
            output = controller.update(currPosition); //Update PID

            //Stop the motor when at rest on the floor
            if (setPosition == 0 && currPosition < liftInchesToTicks(2)) {
                output = 0;
            }
            if (output != lastOutput) {
                lift.setPower(output);

                lastOutput = output;
            }
        }

        //Use motor power to move lift
        else {
            if (liftPower != lastLiftPower) {
                lift.setPower(liftPower);

                lastLiftPower = liftPower;
            }
        }
    }

    /**
     * Stops the lift
     */
    public void stop() {
        setPower(0);
        update();
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

    /**
     * Initializes the PID controller
     */
    private void initPID() {
        controller = new PIDFController(pidCoeffs);
        controller.setOutputBounds(-1, 1);
    }

    /**
     * Initializes class variables to defaults
     */
    private void initVars() {
        output = 0; lastOutput = 0;
        setPosition = 0;
        liftPower = 0; lastLiftPower = 0;
        usePID = true;
    }
}
