package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    //Intake motor
    public DcMotor intake;
    public DcMotor ramp;

    //What power to set to the intake motor
    private double intakePower, lastIntakePower;
    private double rampPower, lastRampPower;

    /**
     * Initialize the class variables and intake and ramp motors
     * @param hardwareMap The hardwareMap is for the motor configuration
     * @param intakeMotorName The name of the intake motor in the configuration
     * @param rampMotorName The name of the ramp motor in the configuration
     */
    public Intake(HardwareMap hardwareMap, String intakeMotorName, String rampMotorName) {
        intake = hardwareMap.dcMotor.get(intakeMotorName);
        ramp = hardwareMap.dcMotor.get(rampMotorName);

        intakePower = 0;
        lastIntakePower = 0;
        rampPower = 0;
        lastRampPower = 0;
    }

    /**
     * Sets power value for both motors
     * @param power The power to set to both motors
     */
    public void setPower(double power) {
        intakePower = power;
        rampPower = power;
    }

    /**
     * Sets unique power values for each motor
     * @param intakePower The power for the intake motor
     * @param rampPower The power for the ramp motor
     */
    public void setPower(double intakePower, double rampPower) {
        this.intakePower = intakePower;
        this.rampPower = rampPower;
    }

    /**
     * Updates motor powers when needed
     */
    public void update() {
        if (intakePower != lastIntakePower) {
            intake.setPower(intakePower);
        }

        if (rampPower != lastRampPower) {
            ramp.setPower(rampPower);
        }

        lastIntakePower = intakePower;
        lastRampPower = rampPower;
    }

    /**
     * Stops the intake and ramp motors
     */
    public void stop() {
        setPower(0);
        update();
    }
}
