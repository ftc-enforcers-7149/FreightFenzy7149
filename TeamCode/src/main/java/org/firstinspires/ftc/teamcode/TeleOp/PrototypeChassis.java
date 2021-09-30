package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;

@TeleOp(name = "Prototype Chassis")
//@Disabled
public class PrototypeChassis extends TeleOp_Base {

    //Headless
    protected Gyroscope gyro;
    private boolean resetAngle;

    //Lift
    private DcMotor lift;
    private double liftPower, lastLiftPower;

    //Intake
    private enum IntakeDirection {
        LEFT, RIGHT, CENTER, NONE
    }

    private Servo wrist;
    private CRServo intake;
    private IntakeDirection intakeDir, lastIntakeDir;
    private double intakePower, lastIntakePower;

    @Override
    public void init() {
        initializeDrive();
        initializeVars();

        gyro = new Gyroscope(hardwareMap);

        lift = hardwareMap.dcMotor.get("lift");
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wrist = hardwareMap.servo.get("wrist");
        wrist.setDirection(Servo.Direction.REVERSE);
        intake = hardwareMap.crservo.get("intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        lastLiftPower = 0;
        lastIntakePower = 0;
        lastIntakeDir = IntakeDirection.NONE;
        intakeDir = IntakeDirection.NONE;
    }

    @Override
    public void loop() {
        getInput();

        driveHeadless(gyro.getRawYaw(), resetAngle);

        //Lift
        if (liftPower != lastLiftPower) {
            lift.setPower(liftPower);
        }

        //Intake
        if (intakeDir != lastIntakeDir) {
            switch (intakeDir) {
                case LEFT:
                    wrist.setPosition(0);
                    break;
                case RIGHT:
                    wrist.setPosition(1);
                    break;
                case CENTER:
                    wrist.setPosition(0.615);
                    break;
            }
        }

        if (intakePower != lastIntakePower) {
            intake.setPower(intakePower);
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
        liftPower = gamepad1.right_trigger - gamepad1.left_trigger;

        //Intake
        if (gamepad2.dpad_up) intakeDir = IntakeDirection.CENTER;
        else if (gamepad2.dpad_left) intakeDir = IntakeDirection.LEFT;
        else if (gamepad2.dpad_right) intakeDir = IntakeDirection.RIGHT;
        intakePower = (gamepad2.right_bumper ? 1 : 0) - (gamepad2.left_bumper ? 1 : 0);
    }

    @Override
    protected void updateStateMachine() {
        lastLeftX = leftX; lastLeftY = leftY; lastRightX = rightX;
        lastLiftPower = liftPower;
        lastIntakeDir = intakeDir;
        lastIntakePower = intakePower;
    }
}
