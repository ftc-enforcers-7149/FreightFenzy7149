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

public class TurretOld implements Input, Output {

    //Turret motor
    public DcMotorEx turret;

    //Bulk Read
    private BulkRead bRead;
    private boolean useBRead;

    //Convert motor ticks to rotations (using Gobilda's given equation)
    private final double toRot = (((1+(46.0/17))) * (1+(46.0/17))) * 28; //TODO: Get right eq.
    private final double CHAIN_GEARING = 56.0/10; // Turret sprocket / motor sprocket
    private final double anglePerTick = toRot * 2 * Math.PI / CHAIN_GEARING;
    private final double ticksPerAngle = CHAIN_GEARING / (toRot * 2 * Math.PI);
    private final double ticksPerRotation = angleToTicks(2 * Math.PI); //Per output rotation

    //Limits
    private final double LIMIT_RANGE = Math.PI / 4;
    private boolean limitRotation;

    //PIDF Controller
    private PIDFController controller;
    private double output, lastOutput;
    //TODO: Tune turret PID
    public static PIDCoefficients pidCoeffs = new PIDCoefficients(0.01, 0, 0);

    //Position (in motor ticks) to run to
    private int currPosition, setPosition;
    private double setAngle;

    //Power to use if not running PID
    private double turretPower, lastTurretPower;

    private boolean usePID;
    private boolean manualOverride;

    public TurretOld(HardwareMap hardwareMap, String turretName) {
        turret = hardwareMap.get(DcMotorEx.class, turretName);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);

        initPID();
        initVars();

        useBRead = false;
    }

    public TurretOld(HardwareMap hardwareMap, String turretName, BulkRead bRead) {
        turret = hardwareMap.get(DcMotorEx.class, turretName);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);

        initPID();
        initVars();

        this.bRead = bRead;
        useBRead = true;
    }

    public TurretOld(HardwareMap hardwareMap, String turretName, BulkRead bRead, boolean reset) {
        turret = hardwareMap.get(DcMotorEx.class, turretName);
        if (reset)
            turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);

        initPID();
        initVars();

        this.bRead = bRead;
        useBRead = true;
    }

    @Override
    public void updateInput() {
        //Get motor ticks
        if (useBRead) currPosition = -bRead.getMotorPos(turret);
        else currPosition = turret.getCurrentPosition();
    }

    @Override
    public void updateOutput() {
        //Use PID to go to position
        if (usePID && !manualOverride) {
            output = controller.update(currPosition); //Update PID

            if (output != lastOutput) {
                turret.setPower(output);

                lastOutput = output;
            }
        }

        //Use motor power to move elevator
        else {
            //TODO: Set limit bounds
            if (!manualOverride && turretPower < 0 && getAngle() < -0.05) turretPower = 0;

            if (turretPower != lastTurretPower) {
                turret.setPower(turretPower);

                lastTurretPower = turretPower;
            }
        }
    }

    /**
     * Set power to the turret motor, stopping the PID
     * @param power Motor power
     */
    public void setPower(double power) {
        turretPower = power;

        if (usePID) lastTurretPower = -2; //Impossible value so power != lastPower
        usePID = false;
    }

    /**
     * Set a target angle for the turret, starting the PID
     * @param angle Target angle in radians
     */
    public void setTargetAngle(double angle) {
        //Convert angle to [-PI, PI]
        setAngle = angle % 2 * Math.PI;
        if (setAngle > Math.PI) setAngle -= 2 * Math.PI;

        //Clip angle so it won't hit the limit
        if (limitRotation) {
            if (Math.abs(setAngle) > Math.PI - LIMIT_RANGE / 2) {
                if (setAngle >= 0) setAngle = Math.PI - LIMIT_RANGE / 2;
                else setAngle = -Math.PI + LIMIT_RANGE / 2;
            }
        }

        //Get smallest difference in position
        double angleDifference = setAngle - getAngle();
        if (angleDifference > Math.PI) angleDifference -= 2 * Math.PI;
        else if (angleDifference < -Math.PI) angleDifference += 2 * Math.PI;
        if (Math.abs(angleDifference) == Math.PI) {
            if (getAngle() > 0) angleDifference = -Math.PI;
            else if (getAngle() < 0) angle = Math.PI;
        }

        double difference = angleToTicks(angleDifference);

        if (limitRotation) {
            //TODO: Figure out if angleDifference goes past limit
        }

        setPosition = currPosition + (int) difference;
        controller.setTargetPosition(setPosition);
        usePID = true;
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
     * Gets the current angle of the turret
     * @return Angle in radians
     */
    public double getAngle() {
        return ticksToAngle(currPosition);
    }

    /**
     * Converts motor ticks to the angle of rotation
     * @param ticks Motor ticks
     * @return Angle of rotation [-PI, PI]
     */
    private double ticksToAngle(int ticks) {
        double angle = (ticks * anglePerTick) % (2 * Math.PI);
        if (angle > Math.PI) angle -= 2 * Math.PI;
        return angle;
    }

    /**
     * Converts angle of rotation to motor ticks
     * @param angle Angle of rotation
     * @return Motor ticks
     */
    private int angleToTicks(double angle) {
        return (int) (angle * ticksPerAngle);
    }

    private void initPID() {
        controller = new PIDFController(pidCoeffs);
        controller.setOutputBounds(-1, 1);
    }

    private void initVars() {
        output = 0; lastOutput = 0;
        setPosition = 0;
        turretPower = 0; lastTurretPower = 0;
        usePID = true;
        manualOverride = false;
        limitRotation = true;
    }

    @Override
    public void stopOutput() {
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        setPower(0);
        updateOutput();
    }
}
