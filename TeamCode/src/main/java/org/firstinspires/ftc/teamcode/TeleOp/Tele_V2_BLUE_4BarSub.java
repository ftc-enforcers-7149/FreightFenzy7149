package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.FourBar;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.MotorCarouselSpinner;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.MotorIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.LED.LED;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Levels;

import static org.firstinspires.ftc.teamcode.GlobalData.HEADING;
import static org.firstinspires.ftc.teamcode.GlobalData.RAN_AUTO;

@TeleOp (name = "BLUE 4BarSub TeleOp")
@Disabled
public class Tele_V2_BLUE_4BarSub extends TeleOp_Base {

    //Drive
    private boolean resetAngle;
    private boolean sharedBarrier, lastSharedBarrier;

    //Control objects
    private MotorIntake intake;
    private Lift lift;
    private MotorCarouselSpinner spinner;
    private LED led;
    private FourBar bar;

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
    private boolean outDuck, lastOutDuck;
    private boolean score, lastScore;

    //Spinner
    private boolean spin, lastSpin;

    private boolean fbIn, fbHalf, fbOut;
    private boolean lastFBIn, lastFBHalf, lastFBOut;

    private double curr4BPos, last4BPos;

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
        lift.setCurrPosition(-128);
        spinner = new MotorCarouselSpinner(hardwareMap, "spinner", Alliance.BLUE);

        led = new LED(hardwareMap, "blinkin", Alliance.BLUE);
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);

        bar = new FourBar(hardwareMap, "fourBarL", "fourBarR", "counterL", "counterR");

        if (RAN_AUTO) gyro.setOffset(HEADING);
        RAN_AUTO = false;

        addInput(intake);
        addInput(lift);
        addInput(spinner);
        addOutput(intake);
        addOutput(lift);
        addOutput(spinner);
        addOutput(led);
        addOutput(bar);

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
        if (sharedBarrier && !lastSharedBarrier)
            startSharedBarrierForward();
        if (leftX != lastLeftX || leftY != lastLeftY || rightX != lastRightX || resetAngle)
            cancelAutomatedDriving();
        if (!isAutomatedDriving())
            driveHeadless(gyro.getYaw(), resetAngle);

        boolean overrideIntakeDropLift = false;
        boolean overrideIntakeSharedBarrier = isAutomatedDriving();

        //Lift
        if (high)
            liftPos = Levels.MIDDLE;
        //if (mid)
        //    liftPos = Levels.MIDDLE;
        //if (low)
        //    liftPos = Levels.LOW;
        //if (cap)
        //    liftPos = Levels.CAP;
        //if (shared)
        //    liftPos = Levels.SHARED;
        if (ground) {
            liftPos = Levels.GROUND;
            if (lastLiftPos == Levels.LOW)
                overrideIntakeDropLift = true;
        }
        else if (!score && !outtake && !in) intake.setIntakePower(0);

        //Set height
        if (liftPower != lastLiftPower) {
            lift.setPower(liftPower);
            lastPowerManual = true;
            liftPos = Levels.MAX;
            lastLiftPos = Levels.MAX;
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

        if (outDuck) {
            intake.cancelSpitOut();
            if (liftPos == Levels.LOW) {
                intake.setIntakePower(0.3);
                intake.setPaddle(MotorIntake.PaddlePosition.OUT_FAR);
            }
            else {
                intake.setIntakePower(0);
                intake.setPaddle(MotorIntake.PaddlePosition.OUT_CLOSE);
            }
            intake.setLatch(MotorIntake.LatchPosition.OPEN_UP);
        }
        else if (lastOutDuck) {
            intake.setIntakePower(0);
            intake.setPaddle(MotorIntake.PaddlePosition.BACK);
            intake.setLatch(MotorIntake.LatchPosition.OPEN);
        }

        if (overrideIntakeDropLift) intake.setIntakePower(-0.5);
        if (overrideIntakeSharedBarrier) intake.setIntakePower(0.2);

        // Carousel
        if (gamepad1.x) spinner.reset();

        //4Bar
        if (fbIn) curr4BPos = 0;
        else if (fbHalf) curr4BPos = 0.2;
        else if (fbOut) curr4BPos = 0.75;
        else if (lastFBOut) curr4BPos = 0;

        //adam was here :D

        if (curr4BPos != last4BPos) {
            bar.setPosition(curr4BPos);
        }

        // Telemetry
        telemetry.addData("Lift Height: ", lift.getHeight());
        telemetry.addData("Freight in Intake: ", freightInIntake);
        telemetry.addData("Four Bar Angle: ", Math.toDegrees(bar.getCurrAngle()));

        // Led
        if (freightInIntake)
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        else
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

        //led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);

        updateAutomatedDriving();
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
        leftX = curveInput(gamepad1.left_stick_x, 1)*lim * 0.92;
        leftY = curveInput(gamepad1.left_stick_y, 1)*lim * 0.92;
        rightX = curveInput(gamepad1.right_stick_x, 1)*lim*0.75 * 0.92;
        resetAngle = gamepad1.y;
        sharedBarrier = gamepad1.a && !gamepad1.start;

        //Lift
        high = gamepad2.dpad_up || gamepad1.dpad_up;
        //mid = gamepad2.dpad_left;
        //low = gamepad2.dpad_down;
        ground = gamepad2.dpad_down || gamepad1.dpad_down;
        //cap = gamepad2.y;
        //shared = gamepad2.dpad_right;

        liftPower = 0.67 * (gamepad2.right_trigger - gamepad2.left_trigger);

        resetLift = gamepad2.back;

        //Intake
        freightInIntake = intake.getFreightInIntake();

        in = gamepad1.right_trigger > 0.2;
        outtake = gamepad1.right_bumper;
        outDuck = gamepad1.left_bumper;
        score = gamepad1.left_trigger > 0.2;

        //Spinner
        spin = gamepad1.x;

        curr4BPos -= gamepad2.right_stick_y / 20; //Fine tune or adjust for actual time changes
        if (curr4BPos < 0) curr4BPos = 0;
        else if (curr4BPos > 1) curr4BPos = 1;

        //4Bar
        fbIn = gamepad2.right_stick_button;
        fbHalf = gamepad2.x; //TODO: Figure out controls
        fbOut = gamepad2.b && !gamepad2.start;
    }

    @Override
    protected void updateStateMachine() {
        //Drive
        lastLeftX = leftX; lastLeftY = leftY; lastRightX = rightX;
        lastSharedBarrier = sharedBarrier;

        //Lift
        lastLiftPower = liftPower;
        lastResetLift = resetLift;
        lastLiftPos = liftPos;

        //Intake
        lastFreightInIntake = freightInIntake;
        lastIn = in;
        lastOuttake = outtake;
        lastOutDuck = outDuck;
        lastScore = score;

        //Spinner
        lastSpin = spin;

        //4Bar
        last4BPos = curr4BPos;

        lastFBIn = fbIn;
        lastFBHalf = fbHalf;
        lastFBOut = fbOut;
    }

    private double scalePos(double pos) {
        double zeroOutput = 0.06;
        double oneOutput = 1;
        return (oneOutput-zeroOutput) * pos  + zeroOutput;
    }
}