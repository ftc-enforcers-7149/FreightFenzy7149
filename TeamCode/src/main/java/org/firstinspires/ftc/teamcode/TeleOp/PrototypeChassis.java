package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;

@TeleOp(name = "Prototype Chassis")
@Disabled
public class PrototypeChassis extends TeleOp_Base {

    //Headless
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

    //Carousel
    private CRServo leftSpinner, rightSpinner;
    private double leftPower, lastLeftPower, rightPower, lastRightPower;

    @Override
    public void init() {
        initializeDrive();
        initializeGyro();
        initializeVars();

        lift = hardwareMap.dcMotor.get("lift");
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wrist = hardwareMap.servo.get("wrist");
        wrist.setDirection(Servo.Direction.REVERSE);
        intake = hardwareMap.crservo.get("intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSpinner = hardwareMap.crservo.get("leftSpinner");
        leftSpinner.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSpinner = hardwareMap.crservo.get("rightSpinner");
        rightSpinner.setDirection(DcMotorSimple.Direction.FORWARD);

        lastLiftPower = 0;
        lastIntakePower = 0;
        lastIntakeDir = IntakeDirection.NONE;
        intakeDir = IntakeDirection.NONE;
        lastLeftPower = 0;
        lastRightPower = 0;
    }

    @Override
    public void loop() {
        gyro.update();
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
                    wrist.setPosition(0.45);
                    break;
            }
        }

        if (intakePower != lastIntakePower) {
            intake.setPower(intakePower);
        }

        if (leftPower != lastLeftPower) {
            leftSpinner.setPower(leftPower);
        }

        if (rightPower != lastRightPower) {
            rightSpinner.setPower(rightPower);
        }

        updateStateMachine();
    }

    @Override
    public void stop() {
        setMotorPowers(0, 0, 0, 0);
        lift.setPower(0);
        intake.setPower(0);
        leftSpinner.setPower(0);
        rightSpinner.setPower(0);
    }

    @Override
    protected void getInput() {
        //Headless
        leftX = gamepad1.left_stick_x;
        leftY = gamepad1.left_stick_y;
        rightX = curveInput(gamepad1.right_stick_x, 7);
        resetAngle = gamepad1.y;

        //Lift
        if (gamepad1.right_trigger > 0.1 || gamepad1.left_trigger > 0.1)
            liftPower = gamepad1.right_trigger - gamepad1.left_trigger;
        else
            liftPower = 0;

        //Intake
        if (gamepad2.dpad_up) intakeDir = IntakeDirection.CENTER;
        else if (gamepad2.dpad_left) intakeDir = IntakeDirection.LEFT;
        else if (gamepad2.dpad_right) intakeDir = IntakeDirection.RIGHT;
        if (gamepad2.right_trigger > 0.1 || gamepad2.left_trigger > 0.1)
            intakePower = gamepad2.right_trigger - (gamepad2.left_trigger * 0.75);
        else
            intakePower = 0;

        //Spinner
        leftPower = gamepad1.x ? 1 : 0;
        rightPower = gamepad1.b ? 1 : 0;
    }

    @Override
    protected void updateStateMachine() {
        lastLeftX = leftX; lastLeftY = leftY; lastRightX = rightX;
        lastLiftPower = liftPower;
        lastIntakeDir = intakeDir;
        lastIntakePower = intakePower;
        lastLeftPower = leftPower;
        lastRightPower = rightPower;
    }
}
