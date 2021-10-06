package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Carrousel {

    public CRServo carrousel;
    private double carrouselPower, lastCarrouselPower;

    public Carrousel(HardwareMap hardwaremap, String intakeServoName) {
        carrousel = hardwaremap.crservo.get(intakeServoName);

        carrousel.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void update() {
        if (carrouselPower != lastCarrouselPower) {
            carrousel.setPower(carrouselPower);
        }

        lastCarrouselPower = carrouselPower;
    }

    public void setPower(double power) {
        carrouselPower  = power;
    }

    public void stop() {
        carrouselPower = 0;
        update();
    }
}
