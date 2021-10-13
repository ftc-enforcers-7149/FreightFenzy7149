package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CarouselSpinner {

    public CRServo carousel;
    private double carouselPower, lastCarouselPower;

    public CarouselSpinner(HardwareMap hardwaremap, String intakeServoName) {
        carousel = hardwaremap.crservo.get(intakeServoName);

        carousel.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void update() {
        if (carouselPower != lastCarouselPower) {
            carousel.setPower(carouselPower);
        }

        lastCarouselPower = carouselPower;
    }

    public void setPower(double power) {
        carouselPower = power;
    }

    public void stop() {
        setPower(0);
        update();
    }
}
