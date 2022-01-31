package org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Utils.Output;

public class CarouselSpinner implements Output {

    //The two spinner servos
    public DcMotorEx spinner;

    //State machine logic
    private double power, lastPower;

    public CarouselSpinner(HardwareMap hardwaremap, String spinnerName) {
        spinner = hardwaremap.get(DcMotorEx.class, spinnerName);

        spinner.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void updateOutput() {
        if (power != lastPower) {
            spinner.setPower(power);
        }

        lastPower = power;
    }

    /**
     * Set the power for the spinner
     * @param power Servo power [-1,1]
     */
    public void setPower(double power) {
        this.power = power;
    }

    @Override
    public void stopOutput() {
        setPower(0);
        updateOutput();
    }
}
