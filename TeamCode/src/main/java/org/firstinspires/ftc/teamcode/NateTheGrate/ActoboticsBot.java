package org.firstinspires.ftc.teamcode.NateTheGrate;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Actobotics Robot")
@Disabled
public class ActoboticsBot extends OpMode {
    DcMotor motorRight, motorLeft;
    public void init() {
        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorLeft = hardwareMap.dcMotor.get("motorLeft");

        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void loop() {
        if (gamepad1.left_stick_y >= 0.1) {
            motorLeft.setPower(1);
        } else if (gamepad1.left_stick_y <= -0.1) {
            motorLeft.setPower(-1);
        } else if (gamepad1.left_stick_y == 0) {motorLeft.setPower(0);}

        if (gamepad1.right_stick_y >= 0.1) {
            motorRight.setPower(1);
        } else if (gamepad1.right_stick_y <= -0.1) {
            motorRight.setPower(-1);
        } else if (gamepad1.left_stick_y == 0) {motorRight.setPower(0);}
    }

    public void stop() {
        motorLeft.setPower(0);
        motorRight.setPower(0);
    }
}