package org.firstinspires.ftc.teamcode.Testing.Prototyping;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Adam Four Bar")
@Disabled
public class AdamFourBar extends OpMode {

   private DcMotor rotate, lift;
   private CRServo intake;

   boolean lRotate, rRotate, liftUp, liftDown, inIntake, outIntake;

   public void init() {

    rotate = hardwareMap.get(DcMotor.class, "rotate");
    lift = hardwareMap.get(DcMotor.class, "lift");

    intake = hardwareMap.crservo.get("intake");

}

   public void loop() {

       lRotate = gamepad1.left_trigger > .10;
       rRotate = gamepad1.right_trigger > .10;
       liftUp = gamepad1.dpad_up;
       liftDown = gamepad1.dpad_down;
       inIntake = gamepad1.y;
       outIntake = gamepad1.a;

       if(lRotate && !rRotate) rotate.setPower(.3);
       else if(rRotate && !lRotate) rotate.setPower(-.3);
       else rotate.setPower(0);

       if(liftDown && !liftUp) lift.setPower(-0.2);
       else if(liftUp && !liftDown) lift.setPower(0.2);
       else lift.setPower(0);

       if(gamepad1.a) intake.setPower(0.7);
       else if(gamepad1.y) intake.setPower(-0.7);
       else intake.setPower(0);

   }

}