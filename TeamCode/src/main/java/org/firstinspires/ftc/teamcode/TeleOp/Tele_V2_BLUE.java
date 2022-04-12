package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.ArmController;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.FourBar;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.MotorCarouselSpinner;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.MotorIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors.ColorSensorFreight;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.LED.LED;

import static org.firstinspires.ftc.teamcode.GlobalData.HEADING;
import static org.firstinspires.ftc.teamcode.GlobalData.RAN_AUTO;

@TeleOp(name = "TeleOp BLUE")
//@Disabled
public class Tele_V2_BLUE extends TeleOp_Base {

    //Control objects
    private MotorIntake intake;
    private Lift lift;
    private FourBar fourBar;
    private ArmController armController;
    private MotorCarouselSpinner spinner;
    private LED led;

    //Drive
    private boolean resetAngle;
    private boolean sharedBarrier, lastSharedBarrier;

    //Lift
    private double liftPower, lastLiftPower;
    private boolean lastPowerManual;

    private boolean resetLift, lastResetLift;
    private boolean manualOverride;

    //Intake
    private boolean freightInIntake, lastFreightInIntake;
    private boolean outtake, lastOuttake, in, lastIn;
    private boolean outDuck, lastOutDuck;
    private boolean score, lastScore;
    private boolean intakeSlow, lastIntakeSlow;

    //Spinner
    private boolean spin, lastSpin;

    //Four Bar
    private double curr4BPos, last4BPos;

    private ArmController.ScoringPosition scorePos = ArmController.ScoringPosition.IN,
            lastScorePos = ArmController.ScoringPosition.IN;
    private boolean gStartA, gStartB, lastGStartA, lastGStartB;
    private boolean capping, capButton, lastCapButton;

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
        fourBar = new FourBar(hardwareMap, "fourBarL", "fourBarR",
                "counterL", "counterR");
        armController = new ArmController(lift, fourBar);
        spinner = new MotorCarouselSpinner(hardwareMap, "spinner", Alliance.BLUE);

        led = new LED(hardwareMap, "blinkin", Alliance.BLUE);
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_BLUE);

        if (RAN_AUTO) gyro.setOffset(HEADING);
        RAN_AUTO = false;

        addInput(intake);
        addInput(lift);
        addInput(spinner);
        addOutput(intake);
        addOutput(lift);
        addOutput(fourBar);
        addOutput(spinner);
        addOutput(led);

        led.startOutput();
        led.updateOutput();
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

        //Set height
        if (liftPower != lastLiftPower) {
            lift.setPower(liftPower);
            lastPowerManual = true;
            scorePos = ArmController.ScoringPosition.IDLE;
        }
        else if (lastPowerManual && liftPower == 0) {
            lift.setTargetHeight(lift.getHeight());
            lastPowerManual = false;
        }

        //Reset
        if (resetLift && !lastResetLift) {
            lift.setManualOverride(!manualOverride);
            manualOverride = !manualOverride;
            scorePos = ArmController.ScoringPosition.IDLE;
        }

        if (curr4BPos != last4BPos) {
            fourBar.setPosition(curr4BPos);
            scorePos = ArmController.ScoringPosition.IDLE;
        }

        // Four Bar
        if (scorePos != lastScorePos && scorePos != ArmController.ScoringPosition.IDLE) {
            armController.setScorePos(scorePos);
            curr4BPos = scorePos.barPos;
        }

        if (scorePos == ArmController.ScoringPosition.IN && lift.getHeight() > ArmController.ScoringPosition.IN.liftPos + 3)
            overrideIntakeDropLift = true;

        // Intake
        if (in && freightInIntake && !lastFreightInIntake) {
            intake.cancelSpitOut();
            intake.setIntakePower(0);
            intake.setPaddle(MotorIntake.PaddlePosition.BACK);
            intake.setLatch(MotorIntake.LatchPosition.CLOSED);

            //Automatically move arm up
            //if (scorePos == ArmController.ScoringPosition.IN)
            //    armController.setScorePos(ArmController.ScoringPosition.UP);
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
            intake.setIntakePower(0);
            intake.setPaddle(MotorIntake.PaddlePosition.OUT_CLOSE);
            intake.setLatch(MotorIntake.LatchPosition.OPEN);
        }
        else if (lastScore) {
            intake.setIntakePower(0);
            intake.setPaddle(MotorIntake.PaddlePosition.BACK);
            intake.setLatch(MotorIntake.LatchPosition.OPEN);
        }

        if (outDuck) {
            intake.cancelSpitOut();
            intake.setIntakePower(0);
            intake.setPaddle(MotorIntake.PaddlePosition.OUT_CLOSE);
            intake.setLatch(MotorIntake.LatchPosition.OPEN_UP);
        }
        else if (lastOutDuck) {
            intake.setIntakePower(0);
            intake.setPaddle(MotorIntake.PaddlePosition.BACK);
            intake.setLatch(MotorIntake.LatchPosition.OPEN);
        }

        if (intakeSlow)
            intake.setIntakePower(0.4);
        else if (lastIntakeSlow)
            intake.setIntakePower(0);

        if (overrideIntakeDropLift) intake.setIntakePower(-0.75);
        else if (intake.getIntakePower() == -0.75) intake.setIntakePower(0);
        if (overrideIntakeSharedBarrier) intake.setIntakePower(0.2);
        else if (intake.getIntakePower() == 0.2) intake.setIntakePower(0);

        // Carousel
        if (spin) spinner.reset();

        // Telemetry
        telemetry.addData("Score Position: ", scorePos);
        //telemetry.addData("Freight in Intake: ", freightInIntake);
        telemetry.addData("Freight type: ", intake.getFreightType());

        // Led
        if (freightInIntake && intake.getCurrPaddle() != MotorIntake.PaddlePosition.OUT_FAR)
            // led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            led.setPattern(intake.getFreightType() == ColorSensorFreight.Freight.BALL ? RevBlinkinLedDriver.BlinkinPattern.WHITE : RevBlinkinLedDriver.BlinkinPattern.GOLD);
        else
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

        updateAutomatedDriving();
        updateOutputs();
        updateStateMachine();
    }

    @Override
    protected void getInput() {
        if (gamepad1.start || lastGStartA) gStartA = gamepad1.a;
        if (gamepad2.start || lastGStartB) gStartB = gamepad2.b;

        //Drive
        leftX = curveInput(gamepad1.left_stick_x, 1)*lim * 1.0;
        leftY = curveInput(gamepad1.left_stick_y, 1)*lim * 1.0;
        rightX = curveInput(gamepad1.right_stick_x, 1)*lim*0.75 * 1.0;
        resetAngle = gamepad1.y;
        sharedBarrier = gamepad1.a && !gStartA;

        capButton = gamepad2.left_bumper;

        //Lift

        if (capButton)
            scorePos = ArmController.ScoringPosition.CAP_PICKUP;

        if (gamepad2.dpad_up || gamepad1.dpad_up)
            scorePos = ArmController.ScoringPosition.UP;
        else if (gamepad2.dpad_down || gamepad1.dpad_down)
            scorePos = ArmController.ScoringPosition.IN;
        else if (gamepad2.dpad_right || gamepad1.dpad_right)
            scorePos = ArmController.ScoringPosition.HIGH;
        else if (gamepad2.dpad_left || gamepad1.dpad_left)
            scorePos = ArmController.ScoringPosition.MIDDLE;
        else if (gamepad2.y)
            scorePos = ArmController.ScoringPosition.FAR;
        else if (gamepad2.b && !gStartB)
            scorePos = ArmController.ScoringPosition.CENTER;
        else if (gamepad2.a)
            scorePos = ArmController.ScoringPosition.CLOSE;
        else if (gamepad2.x)
            scorePos = ArmController.ScoringPosition.REACH;
        else if (scorePos != ArmController.ScoringPosition.IN &&
                scorePos != ArmController.ScoringPosition.CAP_PICKUP &&
                scorePos != ArmController.ScoringPosition.IDLE)
            scorePos = ArmController.ScoringPosition.UP;

        liftPower = 0.8 * (gamepad2.right_trigger - gamepad2.left_trigger);

        resetLift = gamepad2.back;

        //Intake
        freightInIntake = intake.getFreightInIntake();

        in = gamepad1.right_trigger > 0.2;
        outtake = gamepad1.right_bumper;
        outDuck = gamepad1.left_bumper;
        score = gamepad1.left_trigger > 0.2;
        intakeSlow = gamepad1.b;

        //Spinner
        spin = gamepad1.x;

        curr4BPos -= gamepad2.right_stick_y / 15; //Fine tune or adjust for actual time changes
        if (curr4BPos < 0) curr4BPos = 0;
        else if (curr4BPos > 1) curr4BPos = 1;

        if (gamepad2.right_stick_button) curr4BPos = 0;

        if (liftPower != 0 || gamepad2.right_stick_y != 0)
            scorePos = ArmController.ScoringPosition.IDLE;
    }

    @Override
    protected void updateStateMachine() {
        lastGStartA = gStartA;
        lastGStartB = gStartB;

        //Drive
        lastLeftX = leftX; lastLeftY = leftY; lastRightX = rightX;
        lastSharedBarrier = sharedBarrier;

        //Lift
        lastCapButton = capButton;

        lastLiftPower = liftPower;
        lastResetLift = resetLift;

        //Intake
        lastFreightInIntake = freightInIntake;
        lastIn = in;
        lastOuttake = outtake;
        lastOutDuck = outDuck;
        lastScore = score;
        lastIntakeSlow = intakeSlow;

        //Spinner
        lastSpin = spin;

        //4Bar
        last4BPos = curr4BPos;

        lastScorePos = scorePos;
    }
}
