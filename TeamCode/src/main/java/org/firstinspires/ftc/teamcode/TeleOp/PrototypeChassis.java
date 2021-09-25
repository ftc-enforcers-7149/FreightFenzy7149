package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;

@TeleOp(name = "Prototype Chassis")
public class PrototypeChassis extends TeleOp_Base {

    //Headless
    protected Gyroscope gyro;
    private boolean resetAngle;

    //Lift
    private DcMotor lift;
    private double power, lastPower;

    @Override
    public void init() {
        initializeDrive();
        initializeVars();

        gyro = new Gyroscope(hardwareMap);

        lift = hardwareMap.dcMotor.get("lift");
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lastPower = 0;
    }

    @Override
    public void loop() {
        getInput();

        driveHeadless(gyro.getRawYaw(), resetAngle);

        //Lift
        if (power != lastPower) {
            lift.setPower(power);
        }

        updateStateMachine();
    }

    @Override
    public void stop() {
        setMotorPowers(0, 0, 0, 0);
        lift.setPower(0);
    }

    @Override
    protected void getInput() {
        //Headless
        leftX = gamepad1.left_stick_x;
        leftY = gamepad1.left_stick_y;
        rightX = Math.signum(gamepad1.right_stick_x) * Math.abs(Math.pow(gamepad1.right_stick_x, 3));
        resetAngle = gamepad1.y;

        //Lift
        power = gamepad1.right_trigger - gamepad1.left_trigger;
    }

    @Override
    protected void updateStateMachine() {
        lastLeftX = leftX; lastLeftY = leftY; lastRightX = rightX;
        lastPower = power;
    }
}
