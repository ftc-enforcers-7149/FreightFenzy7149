package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.ArmController;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.FourBar;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.MotorCarouselSpinner;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.MotorIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.LED.LED;

import static org.firstinspires.ftc.teamcode.GlobalData.HEADING;
import static org.firstinspires.ftc.teamcode.GlobalData.RAN_AUTO;

@TeleOp(name = "TeleOp RED")
//@Disabled
public class Tele_V2_RED extends TeleOp_Base {

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

    //Spinner
    private boolean spin, lastSpin;

    //Four Bar
    private double curr4BPos, last4BPos;

    private ArmController.ScoringPosition scorePos = ArmController.ScoringPosition.IN,
            lastScorePos = ArmController.ScoringPosition.IN;

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
                "intake", "paddle", "latch", "force");
        lift = new Lift(hardwareMap, "lift", bReadCH, !RAN_AUTO);
        fourBar = new FourBar(hardwareMap, "fourBarL", "fourBarR",
                "counterL", "counterR");
        armController = new ArmController(lift, fourBar);
        spinner = new MotorCarouselSpinner(hardwareMap, "spinner", Alliance.RED);

        led = new LED(hardwareMap, "blinkin", Alliance.RED);
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_RED);

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
        }
        else if (lastPowerManual && liftPower == 0) {
            lift.setTargetHeight(lift.getHeight());
            lastPowerManual = false;
        }

        //Reset
        if (resetLift && !lastResetLift) {
            lift.setManualOverride(!manualOverride);
            manualOverride = !manualOverride;
        }

        // Four Bar
        if (scorePos != lastScorePos && scorePos != ArmController.ScoringPosition.IDLE) {
            armController.setScorePos(scorePos);
        }

        if (scorePos == ArmController.ScoringPosition.IN && lift.getHeight() > scorePos.liftPos + 2)
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

        if (overrideIntakeDropLift) intake.setIntakePower(-0.5);
        if (overrideIntakeSharedBarrier) intake.setIntakePower(0.2);

        // Carousel
        if (spin) spinner.reset();

        // Telemetry
        telemetry.addData("Score Position: ", scorePos);
        telemetry.addData("Freight in Intake: ", freightInIntake);

        // Led
        if (freightInIntake)
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        else
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

        updateAutomatedDriving();
        updateOutputs();
        updateStateMachine();
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

        if (gamepad2.dpad_up || gamepad1.dpad_up)
            scorePos = ArmController.ScoringPosition.UP;
        else if (gamepad2.dpad_down || gamepad1.dpad_down)
            scorePos = ArmController.ScoringPosition.IN;
        else if (gamepad2.dpad_right)
            scorePos = ArmController.ScoringPosition.HIGH;
        else if (gamepad2.dpad_left)
            scorePos = ArmController.ScoringPosition.MIDDLE;
        else if (gamepad2.y)
            scorePos = ArmController.ScoringPosition.FAR;
        else if (gamepad2.b && !gamepad2.start)
            scorePos = ArmController.ScoringPosition.CENTER;
        else if (gamepad2.a)
            scorePos = ArmController.ScoringPosition.CLOSE;
        else if (gamepad2.x)
            scorePos = ArmController.ScoringPosition.REACH;
        else if (scorePos != ArmController.ScoringPosition.IN)
            scorePos = ArmController.ScoringPosition.UP;

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

        if (liftPower != 0 || gamepad2.right_stick_y != 0)
            scorePos = ArmController.ScoringPosition.IDLE;
    }

    @Override
    protected void updateStateMachine() {
        //Drive
        lastLeftX = leftX; lastLeftY = leftY; lastRightX = rightX;
        lastSharedBarrier = sharedBarrier;

        //Lift
        lastLiftPower = liftPower;
        lastResetLift = resetLift;

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

        lastScorePos = scorePos;
    }
}
