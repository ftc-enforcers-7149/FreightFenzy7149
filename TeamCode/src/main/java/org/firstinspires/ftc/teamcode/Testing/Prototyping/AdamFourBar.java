package org.firstinspires.ftc.teamcode.Testing.Prototyping;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Subsystems.Sensors.Encoder;

@TeleOp(name="Adam Four Bar")
public class AdamFourBar extends OpMode {

    private final double TICKS_PER_REV = 8092;
    private final double CHAIN_GEARING = 1d/1d;
    private final double ANGLE_PER_TICK = TICKS_PER_REV * 2 * Math.PI / CHAIN_GEARING;
    private final double TICKS_PER_ANGLE = CHAIN_GEARING / (TICKS_PER_REV * 2 * Math.PI);

    private double currentAngle, desiredAngle = 0;
    private double deadzone = 1; // degree

    private double downAngle = 0, upAngle = 180;

    private DcMotorEx rotate, lift;
    private CRServo intake;
    Encoder liftEnc;

    boolean lRotate, rRotate, liftUp, liftDown, incrementLiftUp, incrementLiftDown, inIntake, outIntake;
    boolean moving = false;

    public void init() {

        rotate = hardwareMap.get(DcMotorEx.class, "rotate");
        lift = hardwareMap.get(DcMotorEx.class, "lift");

        liftEnc = new Encoder(lift);

        intake = hardwareMap.crservo.get("intake");

        rotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void loop() {

        updateInput();

        if(lRotate && !rRotate) rotate.setPower(.3);
        else if(rRotate && !lRotate) rotate.setPower(-.3);
        else rotate.setPower(0);

        if(liftDown && !liftUp && !moving) desiredAngle = 0;
        else if(liftUp && !liftDown && !moving) desiredAngle = 180;

        if(incrementLiftDown && desiredAngle > downAngle && !moving) desiredAngle -= 2;
        else if (incrementLiftUp && desiredAngle < upAngle && !moving) desiredAngle += 2;

        if(gamepad1.a) intake.setPower(0.7);
        else if(gamepad1.y) intake.setPower(-0.7);
        else intake.setPower(0);

        moving = updatePosition();

    }

    public boolean updatePosition() {

        currentAngle = liftEnc.getCurrentPosition() * ANGLE_PER_TICK;

        if(Math.abs(currentAngle - desiredAngle) < deadzone) return false;

        if(desiredAngle < currentAngle) {
            lift.setPower(-0.2);
            return true;
        }
        else if(desiredAngle > currentAngle) {
            lift.setPower(0.2);
            return true;
        }

        return false;

    }

    public void updateInput() {

        lRotate = gamepad1.left_trigger > .10;
        rRotate = gamepad1.right_trigger > .10;
        liftUp = gamepad1.dpad_up;
        liftDown = gamepad1.dpad_down;
        incrementLiftUp = gamepad1.x;
        incrementLiftDown = gamepad1.b;
        inIntake = gamepad1.y;
        outIntake = gamepad1.a;

    }

}