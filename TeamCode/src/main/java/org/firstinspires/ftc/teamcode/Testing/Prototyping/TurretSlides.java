package org.firstinspires.ftc.teamcode.Testing.Prototyping;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Bucket;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Turret;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOp_Base;

@TeleOp(name = "Turret Slides")
public class TurretSlides extends TeleOp_Base {

    //private Intake intake;
    private Elevator elevator;
    private Turret turret;
    private Bucket bucket;

    private CRServo spinner;

    //Elevator control
    private double liftPower, lastLiftPower;
    private Elevator.Level level, lastLevel;
    private boolean resetLift, lastResetLift;
    private boolean manualOverride;
    private boolean holdUp;

    //Turret control
    private double turnPower, lastTurnPower;
    private boolean holdPosition;

    //Bucket control
    private boolean intake, hold, outtake, idle;
    private boolean lastIntake, lastHold, lastOuttake, lastIdle;

    @Override
    public void init() {
        initializeAll();

        elevator = new Elevator(hardwareMap, "elevator", bReadCH);
        turret = new Turret(hardwareMap, "turret", bReadCH);
        bucket = new Bucket(hardwareMap, "intake", "bucket");//, "intakeColor");

        spinner = hardwareMap.crservo.get("spinner");
        spinner.setDirection(DcMotorSimple.Direction.REVERSE);

        elevator.setManualOverride(true);
        turret.setManualOverride(true);

        addInputs(elevator, turret, bucket);
        addOutputs(elevator, turret, bucket);

        level = Elevator.Level.GROUND;
        lastLevel = Elevator.Level.GROUND;
    }

    @Override
    public void loop() {
        updateInputs();
        getInput();

        driveHeadless(gyro.getYaw(), gamepad1.y);
        //driveHeadlessHoldAngle(gamepad1.y);

        //Bucket
        if (!outtake && !intake && !hold && !idle) {
            if (lastOuttake) bucket.outtake(false);
            else if (lastIntake) bucket.intake(false);
        } else if (outtake && !lastOuttake) bucket.outtake(true);
        else if (intake && !lastIntake) bucket.intake(true);
        else if (hold && !lastHold) bucket.holdToggle();
        else if (idle && !lastIdle) bucket.idle();

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
        } else {
            lim = 1;
        }

        //Turret
        if (holdPosition && Math.abs(rightX) >= 0.1 && turnPower == 0) {
            turnPower = -0.6 * rightX;
            if ((turnPower > 0 && turret.turret.getCurrentPosition() < 2 * 3184.02) ||
                    (turnPower < 0 && turret.turret.getCurrentPosition() > -2 * 3184.02)) {
                if (turnPower != lastTurnPower)
                    turret.setPower(turnPower);
            }
            else {
                turnPower = 0;
                if (turnPower != lastTurnPower)
                    turret.setPower(0);
            }
        }
        else if (holdPosition && Math.abs(rightX) >= 0.1 && turnPower != 0) {
            turnPower = (turnPower - 0.6 * rightX) / 2;
            if ((turnPower > 0 && turret.turret.getCurrentPosition() < 2 * 3184.02) ||
                    (turnPower < 0 && turret.turret.getCurrentPosition() > -2 * 3184.02)) {
                if (turnPower != lastTurnPower)
                    turret.setPower(turnPower);
            }
            else {
                turnPower = 0;
                if (turnPower != lastTurnPower)
                    turret.setPower(0);
            }
        }
        else {
            if ((turnPower > 0 && turret.turret.getCurrentPosition() < 2 * 3184.02) ||
                    (turnPower < 0 && turret.turret.getCurrentPosition() > -2 * 3184.02)) {
                if (turnPower != lastTurnPower)
                    turret.setPower(turnPower);
            }
            else {
                turnPower = 0;
                if (turnPower != lastTurnPower)
                    turret.setPower(0);
            }
        }

        //Spinner
        if (gamepad1.b)
            spinner.setPower(1);
        else if (gamepad1.x)
            spinner.setPower(-1);
        else
            spinner.setPower(0);

        telemetry.addData("Turret Angle: ", turret.getAngle());
        telemetry.addData("Elevator Height: ", elevator.getHeight());
        telemetry.addData("Freight Detected? ", bucket.getFreightDetected());

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
        //Drive
        leftX = curveInput(gamepad1.left_stick_x, 2)*lim;
        leftY = curveInput(gamepad1.left_stick_y, 2)*lim;
        rightX = curveInput(gamepad1.right_stick_x, 2)*lim*0.75;

        //Bucket
        intake = gamepad2.right_trigger > 0.1;
        hold = gamepad2.left_bumper;
        outtake = gamepad2.left_trigger > 0.1;
        idle = gamepad2.right_bumper;

        //Elevator
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
        turnPower = gamepad2.left_stick_x * 0.5;
        holdPosition = gamepad2.a;
    }

    @Override
    protected void updateStateMachine() {
        //Drive
        lastLeftX = leftX; lastLeftY = leftY; lastRightX = rightX;

        //Bucket
        lastIntake = intake; lastHold = hold; lastOuttake = outtake; lastIdle = idle;

        //Elevator
        lastLiftPower = liftPower;
        lastLevel = level;
        lastResetLift = resetLift;

        //Turret
        lastTurnPower = turnPower;
    }
}
