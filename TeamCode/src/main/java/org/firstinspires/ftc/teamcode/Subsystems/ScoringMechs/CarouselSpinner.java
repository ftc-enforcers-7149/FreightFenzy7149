package org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Output;

public class CarouselSpinner implements Output {

    //The spinner servo
    public CRServo spinner;

    //State machine logic
    private double power, lastPower;

    public CarouselSpinner(HardwareMap hardwaremap, String spinnerName) {
        spinner = hardwaremap.crservo.get(spinnerName);
        lastPower = 0;
    }

    @Override
    public void updateOutput() {
        if (power != lastPower) {
            spinner.setPower(power);
        }
        lastPower = power;
    }

    /**
     * Set the power for the alliance-specific spinner
     * @param alliance The alliance determines the spinner's direction
     * @param power Servo power [0,1]
     */
    public void setPower(Alliance alliance, double power) {
        if (alliance == Alliance.RED) this.power = -power;
        else if (alliance == Alliance.BLUE) this.power = power;
        else this.power = 0;
    }

    /**
     * Set the power for the spinner (regardless of alliance)
     * @param power Servo power [-1,1]
     */
    public void setPower(double power) {
        this.power = power;
    }

    @Override
    public void stopOutput() {
        power = 0;
        updateOutput();
    }
}
