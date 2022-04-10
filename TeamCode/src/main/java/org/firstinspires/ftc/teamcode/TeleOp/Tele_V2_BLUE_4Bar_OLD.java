package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.MotorCarouselSpinner;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.MotorIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.LED.LED;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Levels;

import static org.firstinspires.ftc.teamcode.GlobalData.HEADING;
import static org.firstinspires.ftc.teamcode.GlobalData.RAN_AUTO;

@TeleOp (name = "BLUE 4Bar TeleOp OLD")
@Disabled
public class Tele_V2_BLUE_4Bar_OLD extends TeleOp_Base {

    //Drive
    private boolean resetAngle;
    private boolean sharedBarrier, lastSharedBarrier;

    //Control objects
    private MotorIntake intake;
    private Lift lift;
    private MotorCarouselSpinner spinner;
    private LED led;

    //4Bar
    private Servo fourBarL, fourBarR;
    private Servo counterL, counterR;

    private Levels liftPos = Levels.GROUND, lastLiftPos = Levels.GROUND;
    private boolean high, ground;
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

    private enum ScoringPosition {
        IN(0, 0, 1),
        UP(7, 0, 1),
        LOW(0, 0.5, 0.6),
        MIDDLE(0, 0.75, 0.6),
        HIGH(4.8, 0.815, 0.6),
        CLOSE(5, 0, 0.6),
        CENTER(4.5, 0.2, 0.6),
        FAR(3.5, 0.35, 0.6),
        REACH(0, 0.6, 0.45);

        double liftPos, barPos, maxSpeed;

        ScoringPosition(double liftPos, double barPos, double maxSpeed) {
            this.liftPos = liftPos;
            this.barPos = barPos;
            this.maxSpeed = maxSpeed;
        }
    }
    private ScoringPosition scorePos = ScoringPosition.IN, lastScorePos = ScoringPosition.IN;

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

        fourBarL = hardwareMap.servo.get("fourBarL");
        fourBarL.setDirection(Servo.Direction.FORWARD);
        fourBarL.setPosition(scalePos(0));
        fourBarR = hardwareMap.servo.get("fourBarR");
        fourBarR.setDirection(Servo.Direction.REVERSE);
        fourBarR.setPosition(scalePos(0));

        counterL = hardwareMap.servo.get("counterL");
        counterL.setDirection(Servo.Direction.REVERSE);
        counterL.setPosition(0);
        counterR = hardwareMap.servo.get("counterR");
        counterR.setDirection(Servo.Direction.FORWARD);
        counterR.setPosition(0);

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
        if (spin) spinner.reset();

        //4Bar
        if (fbIn) curr4BPos = 0;
        else if (fbHalf) curr4BPos = 0.2;
        else if (fbOut) curr4BPos = 0.75;
        else if (lastFBOut) curr4BPos = 0;

        if (scorePos != lastScorePos) {
            lift.setTargetHeight(scorePos.liftPos, scorePos.maxSpeed);

            curr4BPos = scorePos.barPos;
        }
        
        if (curr4BPos != last4BPos) {
            fourBarR.setPosition(scalePos(curr4BPos));
            fourBarL.setPosition(scalePos(curr4BPos));

            counterL.setPosition(curr4BPos);
            counterR.setPosition(curr4BPos);
        }

        // Telemetry
        telemetry.addData("Lift Height: ", lift.getHeight());
        telemetry.addData("Freight in Intake: ", freightInIntake);

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
        leftX = curveInput(gamepad1.left_stick_x, 1)*lim * 0.97;
        leftY = curveInput(gamepad1.left_stick_y, 1)*lim * 0.97;
        rightX = curveInput(gamepad1.right_stick_x, 1)*lim*0.75 * 0.97;
        resetAngle = gamepad1.y;
        sharedBarrier = gamepad1.a && !gamepad1.start;

        //Lift

        if (gamepad2.dpad_up || gamepad1.dpad_up)
            scorePos = ScoringPosition.UP;
        else if (gamepad2.dpad_down || gamepad1.dpad_down)
            scorePos = ScoringPosition.IN;
        else if (gamepad2.dpad_right)
            scorePos = ScoringPosition.HIGH;
        else if (gamepad2.dpad_left)
            scorePos = ScoringPosition.MIDDLE;
        else if (gamepad2.y)
            scorePos = ScoringPosition.FAR;
        else if (gamepad2.b && !gamepad2.start)
            scorePos = ScoringPosition.CENTER;
        else if (gamepad2.a)
            scorePos = ScoringPosition.CLOSE;
        else if (gamepad2.x)
            scorePos = ScoringPosition.REACH;
        else if (scorePos != ScoringPosition.IN)
            scorePos = ScoringPosition.UP;

        liftPower = 0.8 * (gamepad2.right_trigger - gamepad2.left_trigger);

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
        //fbIn = gamepad2.right_stick_button;
        //fbHalf = gamepad2.x; //TODO: Figure out controls
        //fbOut = gamepad2.b && !gamepad2.start;


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

        lastScorePos = scorePos;
    }

    private double scalePos(double pos) {
        double zeroOutput = 0.06;
        double oneOutput = 1;
        return (oneOutput-zeroOutput) * pos  + zeroOutput;
    }
}