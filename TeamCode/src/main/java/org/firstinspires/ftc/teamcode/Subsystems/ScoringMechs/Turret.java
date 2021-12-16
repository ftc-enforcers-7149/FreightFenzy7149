package org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Sensors.BulkRead;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Input;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Output;

public class Turret implements Input, Output {

    //Turret motor
    public DcMotorEx turret;

    //Bulk Read
    public BulkRead bRead;
    private final boolean useBR;

    //Input
    private int currPosition = 0;
    private int offset = 0;

    //Convert motor ticks to rotations (using Gobilda's given equation)
    private final double ticksPerMotorRot = ((((1+(46/17d))) * (1+(46/11d))) * 28);
    private final double CHAIN_GEARING = 56.0/10; // Turret sprocket / motor sprocket
    private final double anglePerTick = ticksPerMotorRot * 2 * Math.PI / CHAIN_GEARING;
    private final double ticksPerAngle = CHAIN_GEARING / (ticksPerMotorRot * 2 * Math.PI);
    private final double ticksPerRotation = angleToTicks(2 * Math.PI); //Per output rotation

    private final double LEFTMOST = -ticksPerRotation * 1.5;
    private final double RIGHTMOST = ticksPerRotation * 1.5;

    private double currAngle = 0;

    //PIDF Controller
    private PIDFController controller;
    private double output, lastOutput;
    public static PIDCoefficients pidCoeffs = new PIDCoefficients(0.01, 0, 0);
    private int setPosition = 0;
    private double setAngle = 0;

    //Power to use if not running PID
    private double turretPower, lastTurretPower;

    //If using PID or not
    private boolean usePID;

    //If using manual override
    private boolean manualOverride;

    public Turret(HardwareMap hardwareMap, String turretName, BulkRead bRead) {
        turret = hardwareMap.get(DcMotorEx.class, turretName);
        this.bRead = bRead;

        initPID();
        initVars();

        useBR = true;
    }

    public Turret(HardwareMap hardwareMap, String turretName) {
        turret = hardwareMap.get(DcMotorEx.class, turretName);

        initPID();
        initVars();

        useBR = false;
    }

    private void setCurrPosition(int position) {
        offset = position - currPosition;
        currPosition = position;
    }

    private void setCurrAngle(double angle) {
        setCurrPosition(angleToTicks(angle));
        if (usePID && !manualOverride) setTargetAngle(setAngle);
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

    private int angleToTicksSmart(double angle) {
        angle = angle % (2 * Math.PI);
        if (angle > Math.PI) angle -= 2 * Math.PI;

        //Get smallest difference in position
        double angleDifference = angle - getAngle();
        if (angleDifference > Math.PI) angleDifference -= 2 * Math.PI;
        else if (angleDifference < -Math.PI) angleDifference += 2 * Math.PI;

        int position = currPosition + angleToTicks(angleDifference);

        if (position < LEFTMOST) return position + (int) ticksPerRotation;
        else if (position > RIGHTMOST) return position - (int) ticksPerRotation;
        else return position;
    }

    @Override
    public void updateInput() {
        //Get motor ticks
        if (useBR) currPosition = -bRead.getMotorPos(turret) + offset;
        else currPosition = turret.getCurrentPosition() + offset;

        currAngle = ticksToAngle(currPosition);
    }

    @Override
    public void updateOutput() {
        //Use PID to go to position
        if (usePID && !manualOverride) {
            output = controller.update(currPosition); //Update PID

            //Stop the motor when at rotation limits
            if ((setPosition <= LEFTMOST && currAngle < LEFTMOST - 0.05) ||
                    (setPosition >= RIGHTMOST && currAngle > RIGHTMOST + 0.05)) {
                output = 0;
            }
            if (output != lastOutput) {
                turret.setPower(output);

                lastOutput = output;
            }
        }

        //Use motor power to move turret
        else {
            if (!manualOverride &&
                    ((turretPower < 0 && currAngle < LEFTMOST - 0.05)
                            || (turretPower > 0 && currAngle > RIGHTMOST + 0.05))) turretPower = 0;

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

        if (power != 0) {
            if (usePID) lastTurretPower = -2; //Impossible value so power != lastPower
            usePID = false;
        }
        else if (!manualOverride) { //Hold position
            setTargetAngle(getAngle());
        }
    }

    /**
     * Set a target angle for the turret, starting the PID
     * @param angle Target angle in radians
     */
    public void setTargetAngle(double angle) {
        //Keep bounds between 0 and MAX_HEIGHT
        setAngle = angle;
        setPosition = angleToTicksSmart(angle);
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
        if (override) setCurrPosition(0);
    }

    public double getAngle() {
        return currAngle;
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
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        setManualOverride(true);
        setPower(0);
        updateOutput();
    }
}
