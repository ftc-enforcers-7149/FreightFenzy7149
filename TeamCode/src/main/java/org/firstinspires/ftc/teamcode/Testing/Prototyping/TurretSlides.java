package org.firstinspires.ftc.teamcode.Testing.Prototyping;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Turret;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOp_Base;

@TeleOp(name = "Turret Slides")
public class TurretSlides extends TeleOp_Base {

    private Intake intake;
    private Elevator elevator;
    private Turret turret;

    private CRServo spinner;

    //Intake control
    private boolean forward, lastForward, backward, lastBackward;

    //Elevator control
    private double liftPower, lastLiftPower;
    private Elevator.Level level, lastLevel;
    private boolean resetLift, lastResetLift;
    private boolean manualOverride;

    //Turret control
    private double angle, lastAngle, turnPower;

    boolean holdUp;

    @Override
    public void init() {
        initializeAll();

        intake = new Intake(hardwareMap, "intake");//, "intakeColor", "outtakeColor");
        elevator = new Elevator(hardwareMap, "elevator", bReadCH);
        turret = new Turret(hardwareMap, "turret", bReadCH);

        spinner = hardwareMap.crservo.get("spinner");
        spinner.setDirection(DcMotorSimple.Direction.REVERSE);

        elevator.setManualOverride(true);
        turret.setManualOverride(true);

        addInputs(intake, elevator, turret);
        addOutputs(intake, elevator, turret);

        lastForward = false; lastBackward = false;
        lastLiftPower = 0;
        level = Elevator.Level.GROUND;
        lastLevel = Elevator.Level.GROUND;
        lastResetLift = false;
        manualOverride = false;
        holdUp = false;
    }

    @Override
    public void loop() {
        updateInputs();
        getInput();

        driveHeadless(gyro.getYaw(), gamepad1.y);

        //Intake
        if (forward && !lastForward) intake.forward();
        else if (backward && !lastBackward) intake.backward();

        // Elevator
        if (liftPower != lastLiftPower)
            if (liftPower != 0)
                elevator.setPower(liftPower);
            else if (holdUp)
                elevator.setPower(0.08);
            else
                elevator.setPower(0);
        else if (level != lastLevel) {
            elevator.setTargetHeight(level);
        }

        if (resetLift && !lastResetLift) {
            elevator.setManualOverride(!manualOverride);
            manualOverride = !manualOverride;
        }

        if (elevator.getHeight() > 10) {
            lim = 0.6;
        }
        else {
            lim = 1;
        }

        //Turret
        //if (angle != lastAngle)
        //    turret.setTargetAngle(angle);

        if (gamepad2.a && Math.abs(gamepad1.right_stick_x) >= 0.3 && turnPower == 0) {
            turnPower = -0.6 * gamepad1.right_stick_x;
            if ((turnPower > 0 && turret.turret.getCurrentPosition() < 2 * 3184.02) ||
                    (turnPower < 0 && turret.turret.getCurrentPosition() > -2 * 3184.02))
                turret.setPower(turnPower);
            else
                turret.setPower(0);
        }
        else if (gamepad2.a && Math.abs(gamepad1.right_stick_x) >= 0.3 && turnPower != 0) {
            turnPower = (turnPower - 0.6 * gamepad1.right_stick_x) / 2;
            if ((turnPower > 0 && turret.turret.getCurrentPosition() < 2 * 3184.02) ||
                    (turnPower < 0 && turret.turret.getCurrentPosition() > -2 * 3184.02))
                turret.setPower(turnPower);
            else
                turret.setPower(0);
        }
        else {
            if ((turnPower > 0 && turret.turret.getCurrentPosition() < 2 * 3184.02) ||
                    (turnPower < 0 && turret.turret.getCurrentPosition() > -2 * 3184.02))
                turret.setPower(turnPower);
            else
                turret.setPower(0);
        }

        if (gamepad1.b)
            spinner.setPower(1);
        else if (gamepad1.x)
            spinner.setPower(-1);
        else
            spinner.setPower(0);

        //telemetry.addData("Elevator Height: ", elevator.getCurrHeightAAA());
        telemetry.addData("Turret Ticks: ", turret.turret.getCurrentPosition());
        telemetry.addData("Turret Angle: ", turret.getAngle());

        updateOutputs();
        updateStateMachine();
    }

    @Override
    public void stop() {
        stopInputs();
        stopOutputs();
    }

    @Override
    protected void getInput() {
        //Headless
        leftX = curveInput(gamepad1.left_stick_x, 2)*lim;
        leftY = curveInput(gamepad1.left_stick_y, 2)*lim;
        rightX = curveInput(gamepad1.right_stick_x, 2)*lim*0.75;

        //Intake
        forward = gamepad2.right_bumper;
        backward = gamepad2.left_bumper;

        //Elevator
        //if (gamepad2.right_trigger > 0.1 || gamepad2.left_trigger > 0.1)
        //    turnPower = gamepad2.right_trigger - gamepad2.left_trigger;
        //else
        //    turnPower = 0;
        turnPower = gamepad2.left_stick_x * 0.5;

        if (gamepad2.dpad_up) level = Elevator.Level.HIGH;
        else if (gamepad2.dpad_left) level = Elevator.Level.MIDDLE;
        else if (gamepad2.dpad_right) level = Elevator.Level.LOW;
        else if (gamepad2.dpad_down) level = Elevator.Level.GROUND;

        if (gamepad1.dpad_up) level = Elevator.Level.HIGH;
        else if (gamepad1.dpad_left) level = Elevator.Level.MIDDLE;
        else if (gamepad1.dpad_right) level = Elevator.Level.LOW;
        else if (gamepad1.dpad_down) level = Elevator.Level.GROUND;

        resetLift = gamepad1.back;

        //Turret
        if (gamepad2.right_stick_y != 0 && gamepad2.right_stick_x != 0)
            angle = Math.atan2(-gamepad2.right_stick_y, gamepad2.right_stick_x) - Math.toRadians(gyro.getYaw());

        if (-gamepad2.left_stick_y > 0.1) {
            liftPower = -gamepad2.left_stick_y;
            holdUp = true;
        }
        else if (-gamepad2.left_stick_y < -0.1) {
            liftPower = -gamepad2.left_stick_y;
            holdUp = false;
        }
        else
            liftPower = 0;
    }

    @Override
    protected void updateStateMachine() {
        lastLeftX = leftX; lastLeftY = leftY; lastRightX = rightX;
        lastForward = forward; lastBackward = backward;
        lastLiftPower = liftPower;
        lastLevel = level;
        lastResetLift = resetLift;
        lastAngle = angle;
    }
}
