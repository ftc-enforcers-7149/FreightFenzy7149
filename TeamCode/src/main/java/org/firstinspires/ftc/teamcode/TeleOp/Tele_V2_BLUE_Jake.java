package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.MotorCarouselSpinner;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.MotorIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.LED.LED;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Levels;

import static org.firstinspires.ftc.teamcode.GlobalData.HEADING;
import static org.firstinspires.ftc.teamcode.GlobalData.RAN_AUTO;

@TeleOp (name = "BLUE Tele_V2 (Jacob)")
//@Disabled
public class Tele_V2_BLUE_Jake extends TeleOp_Base {

    //Headless
    private boolean resetAngle;

    //Control objects
    private MotorIntake intake;
    private Lift lift;
    private MotorCarouselSpinner spinner;
    private LED led;

    //Lift
    public Levels cycleBackward(Levels level) {
        switch (level) {
            case HIGH: return Levels.MIDDLE;
            case MIDDLE: return Levels.LOW;
            case LOW:
            default: return Levels.HIGH;
        }
    }

    private Levels liftPos = Levels.GROUND, lastLiftPos = Levels.GROUND;
    private boolean high, mid, low, ground, cap, shared;
    private double liftPower, lastLiftPower;
    private boolean lastPowerManual;

    private boolean resetLift, lastResetLift;
    private boolean manualOverride;

    //Intake
    private boolean freightInIntake, lastFreightInIntake;
    private boolean outtake, lastOuttake, in, lastIn;
    private boolean score, lastScore;

    //Spinner
    private boolean spin, lastSpin;

    @Override
    public void init() {
        try {
            initializeAll();
        } catch (Exception e) {
            e.printStackTrace();
            requestOpModeStop();
            return;
        }

        intake = new MotorIntake(hardwareMap,
                "intake", "paddle", "latch", "intakeColor");
        lift = new Lift(hardwareMap, "lift", bReadCH, !RAN_AUTO);
        spinner = new MotorCarouselSpinner(hardwareMap, "spinner", Alliance.BLUE);

        led = new LED(hardwareMap, "blinkin", Alliance.BLUE);
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);

        if (RAN_AUTO) gyro.setOffset(HEADING);
        RAN_AUTO = false;

        addInput(intake);
        addInput(lift);
        addInput(spinner);
        addOutput(intake);
        addOutput(lift);
        addOutput(spinner);
        addOutput(led);

        led.startOutput();
        led.updateOutput();
    }

    @Override
    public void init_loop() {
        telemetry.addData("HEADING: ", HEADING);
    }

    @Override
    public void start() {
        startInputs();
        startOutputs();
    }

    @Override
    public void loop() {
        updateInputs();
        getInput();

        // Drive
        driveHeadless(gyro.getYaw(), resetAngle);

        boolean overrideIntake = false;

        //Lift
        if (high)
            liftPos = Levels.HIGH;
        if (mid)
            liftPos = Levels.MIDDLE;
        if (low)
            liftPos = Levels.LOW;
        if (cap)
            liftPos = Levels.CAP;
        if (shared)
            liftPos = Levels.SHARED;
        if (ground) {
            liftPos = Levels.GROUND;
            if (lastLiftPos == Levels.LOW)
                overrideIntake = true;
        }
        else if (!score && !outtake && !in) intake.setIntakePower(0);

        //Set height
        if (liftPower != lastLiftPower) {
            lift.setPower(liftPower);
            lastPowerManual = true;
        }
        else if (lastPowerManual && liftPower == 0) {
            lift.setTargetHeight(lift.getHeight());
            lastPowerManual = false;
        }

        if (liftPos != lastLiftPos) {
            lift.setTargetHeight(liftPos);
            lastPowerManual = false;
        }

        //Reset
        if (resetLift && !lastResetLift) {
            lift.setManualOverride(!manualOverride);
            manualOverride = !manualOverride;
        }

        // Intake
        if (in && freightInIntake && !lastFreightInIntake) {
            intake.cancelSpitOut();
            intake.setIntakePower(0);
            intake.setPaddle(MotorIntake.PaddlePosition.BACK);
            intake.setLatch(MotorIntake.LatchPosition.CLOSED);
            //lift.setTargetHeight(Levels.LOW);
            //lastPowerManual = false;
        }
        else {
            if (in) {
                intake.cancelSpitOut();
                intake.setIntakePower(1);
                intake.setPaddle(MotorIntake.PaddlePosition.BACK);
                intake.setLatch(MotorIntake.LatchPosition.OPEN);
            }
            else if (lastIn) {
                intake.setIntakePower(0);
                intake.setPaddle(MotorIntake.PaddlePosition.BACK);
                intake.setLatch(MotorIntake.LatchPosition.CLOSED);
            }
        }

        /*if (outtake) {
            intake.setIntakePower(-1);
            intake.setPaddle(MotorIntake.PaddlePosition.OUT);
            intake.setLatch(MotorIntake.LatchPosition.OPEN);
        }
        else if (lastOuttake) {
            intake.setIntakePower(0);
            intake.setPaddle(MotorIntake.PaddlePosition.BACK);
            intake.setLatch(MotorIntake.LatchPosition.CLOSED);
        }*/

        if (outtake && !lastOuttake)
            intake.spitOutTwo();

        if (score) {
            intake.cancelSpitOut();
            if (liftPos == Levels.LOW) {
                intake.setIntakePower(0.3);
                intake.setPaddle(MotorIntake.PaddlePosition.OUT_FAR);
            }
            else {
                intake.setIntakePower(0);
                intake.setPaddle(MotorIntake.PaddlePosition.OUT_CLOSE);
            }
            intake.setLatch(MotorIntake.LatchPosition.OPEN);
        }
        else if (lastScore) {
            intake.setIntakePower(0);
            intake.setPaddle(MotorIntake.PaddlePosition.BACK);
            intake.setLatch(MotorIntake.LatchPosition.OPEN);
        }

        if (overrideIntake) intake.setIntakePower(-0.5);

        // Carousel
        if (gamepad1.x) spinner.reset();

        // Telemetry
        telemetry.addData("Lift Height: ", lift.getHeight());
        telemetry.addData("Freight in Intake: ", freightInIntake);

        // Led
        if (freightInIntake)
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        else
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

        //led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);

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
        leftX = curveInput(gamepad1.left_stick_x, 1)*lim * 0.92;
        leftY = curveInput(gamepad1.left_stick_y, 1)*lim * 0.92;
        rightX = curveInput(gamepad1.right_stick_x, 1)*lim*0.75 * 0.92;
        resetAngle = gamepad1.y;

        //Lift
        high = gamepad2.dpad_up;
        mid = gamepad2.dpad_left;
        low = gamepad2.dpad_down;
        ground = gamepad2.a;
        cap = gamepad2.y;
        shared = gamepad2.dpad_right;

        liftPower = gamepad2.right_trigger - gamepad2.left_trigger;

        resetLift = gamepad2.back;

        //Intake
        freightInIntake = intake.getFreightInIntake();

        in = gamepad1.right_trigger > 0.2;
        outtake = gamepad1.right_bumper;
        score = gamepad1.left_trigger > 0.2;

        //Spinner
        spin = gamepad1.x;
    }

    @Override
    protected void updateStateMachine() {
        //Headless
        lastLeftX = leftX; lastLeftY = leftY; lastRightX = rightX;

        //Lift
        lastLiftPower = liftPower;
        lastResetLift = resetLift;
        lastLiftPos = liftPos;

        //Intake
        lastFreightInIntake = freightInIntake;
        lastIn = in;
        lastOuttake = outtake;
        lastScore = score;

        //Spinner
        lastSpin = spin;
    }
}
