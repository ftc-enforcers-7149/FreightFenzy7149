package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CarouselSpinner {

    //The two spinner servos
    public CRServo leftSpinner, rightSpinner;

    //State machine logic
    private double leftPower, lastLeftPower, rightPower, lastRightPower;

    public CarouselSpinner(HardwareMap hardwaremap, String leftName, String rightName) {
        leftSpinner = hardwaremap.crservo.get(leftName);
        rightSpinner = hardwaremap.crservo.get(rightName);

        leftSpinner.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSpinner.setDirection(DcMotorSimple.Direction.FORWARD);

        lastLeftPower = 0;
        lastRightPower = 0;
    }

    public void update() {
        if (leftPower != lastLeftPower) {
            leftSpinner.setPower(leftPower);
        }

        if (rightPower != lastRightPower) {
            rightSpinner.setPower(rightPower);
        }

        lastLeftPower = leftPower;
        lastRightPower = rightPower;
    }

    /**
     * Set the power for the left spinner
     * @param power Servo power [-1,1]
     */
    public void setLeftPower(double power) {
        leftPower = power;
    }

    /**
     * Set the power for the right spinner
     * @param power Servo power [-1,1]
     */
    public void setRightPower(double power) {
        rightPower = power;
    }

    public void stop() {
        setLeftPower(0);
        setRightPower(0);
        update();
    }
}
