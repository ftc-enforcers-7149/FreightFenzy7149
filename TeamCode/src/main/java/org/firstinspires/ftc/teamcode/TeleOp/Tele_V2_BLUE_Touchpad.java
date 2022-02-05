package org.firstinspires.ftc.teamcode.TeleOp;

import com.enforcers7149.touchpadplusplus.src.Utils.Bounds.RectBounds;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Subsystems.EnforcersButton;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.CarouselSpinner;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.MotorIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.LED.LED;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Levels;

import static org.firstinspires.ftc.teamcode.GlobalData.HEADING;
import static org.firstinspires.ftc.teamcode.GlobalData.RAN_AUTO;

@TeleOp (name = "BLUE Tele_V2 TOUCHPAD")
//@Disabled
public class Tele_V2_BLUE_Touchpad extends TeleOp_Base {

    //Headless
    private boolean resetAngle;

    //Control objects
    private MotorIntake intake;
    private Lift lift;
    private CarouselSpinner spinner;
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
    private Levels allianceLevel = Levels.HIGH;

    private boolean toggleAllianceHub, lastToggleAllianceHub;
    private boolean atAllianceLevel;
    private boolean toggleSharedHub, lastToggleSharedHub;
    private boolean atSharedLevel;
    private boolean toggleCapping, lastToggleCapping;
    private boolean capping, capDown;
    private boolean switchAllianceLevel, lastSwitchAllianceLevel;

    private boolean resetLift, lastResetLift;
    private boolean manualOverride;

    //Intake
    private boolean freightInIntake, lastFreightInIntake;
    private boolean outtake, lastOuttake, in, lastIn;
    private boolean stopIntake, lastStopIntake;
    private boolean killSwitch;

    //Spinner
    private boolean spin, lastSpin;

    // Touchpad
    private EnforcersButton spinButton, capButton;

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
        spinner = new CarouselSpinner(hardwareMap, "spinner");

        led = new LED(hardwareMap, "blinkin", Alliance.BLUE);
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);

        spinButton = new EnforcersButton(touchpad1.t, false, new RectBounds(0, 100, -100, 100), true);
        capButton = new EnforcersButton(touchpad1.t, false, new RectBounds(-100, 0, -100, 100), true);

        if (RAN_AUTO) gyro.setOffset(HEADING);
        RAN_AUTO = false;

        addInput(intake);
        addInput(spinButton);
        addInput(capButton);
        addInput(lift);
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

        /*if (lift.getHeight() > 10) {
            lim = 0.9;
        }
        else {
            lim = 1;
        }*/

        //Intake
        if (lastLiftPos == Levels.GROUND && lift.getHeight() <= Levels.GROUND.height + 0.1) {
            if (freightInIntake && !lastFreightInIntake) {
                intake.setIntakePower(0);
                intake.setPaddle(MotorIntake.PaddlePosition.BACK);
                intake.setLatch(MotorIntake.LatchPosition.CLOSED);
                toggleSharedHub = true; //Go to shared level automatically
            } else {
                if (in) {
                    intake.setIntakePower(1);
                    intake.setPaddle(MotorIntake.PaddlePosition.BACK);
                    intake.setLatch(MotorIntake.LatchPosition.OPEN);
                } else if (lastIn) {
                    intake.setPaddle(MotorIntake.PaddlePosition.BACK);
                    intake.setLatch(MotorIntake.LatchPosition.CLOSED);
                }
            }
        }
        else {
            in = false;
        }

        if (outtake) {
            if (liftPos == Levels.LOW)
                intake.setIntakePower(0.333);
            else
                intake.setIntakePower(0);
            intake.setPaddle(MotorIntake.PaddlePosition.OUT);
            intake.setLatch(MotorIntake.LatchPosition.OPEN);
        }
        else if (lastOuttake) {
            intake.setPaddle(MotorIntake.PaddlePosition.BACK);
            intake.setLatch(MotorIntake.LatchPosition.OPEN);
        }

        if (!in && !outtake) intake.setIntakePower(0);

        //Lift

        //Change height
        if (switchAllianceLevel && !lastSwitchAllianceLevel) {
            if (liftPos == Levels.CAP)
                capDown = true;
            else
                allianceLevel = cycleBackward(allianceLevel);
        }

        if (toggleAllianceHub && !lastToggleAllianceHub) {
            atAllianceLevel = (lastLiftPos != allianceLevel && lastLiftPos != Levels.CAP
                    && lastLiftPos != Levels.HIGH);
            atSharedLevel = false;
            capping = false;
        }
        else if (toggleSharedHub && !lastToggleSharedHub) {
            atAllianceLevel = false;
            atSharedLevel = (lastLiftPos != Levels.LOW);
            capping = false;
        }
        else if (toggleCapping && !lastToggleCapping) {
            atAllianceLevel = false;
            atSharedLevel = false;
            capping = (lastLiftPos != Levels.CAP);
        }

        if (capping) {
            if (capDown) {
                liftPos = Levels.HIGH;
                atAllianceLevel = true;
                allianceLevel = Levels.HIGH;
                capping = false;
            }
            else
                liftPos = Levels.CAP;
        }
        else {
            capDown = false;
            if (atAllianceLevel) liftPos = allianceLevel;
            else if (atSharedLevel) liftPos = Levels.LOW;
            else liftPos = Levels.GROUND;
        }

        //Set height
        if (liftPos != lastLiftPos) {
            lift.setTargetHeight(liftPos);
        }

        //Reset
        if (resetLift && !lastResetLift) {
            lift.setManualOverride(!manualOverride);
            manualOverride = !manualOverride;
        }

        // Carousel
        spinner.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
        telemetry.addData("Spinner Power: ", gamepad2.right_trigger - gamepad2.left_trigger);

        // Telemetry
        telemetry.addData("Lift Height: ", lift.getHeight());
        telemetry.addData("Freight in Intake: ", freightInIntake);

        //Led
        if (freightInIntake)
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        else
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

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
        leftX = curveInput(gamepad1.left_stick_x, 1)*lim * 0.75;
        leftY = curveInput(gamepad1.left_stick_y, 1)*lim * 0.75;
        rightX = curveInput(gamepad1.right_stick_x, 1)*lim*0.75 * 0.75;
        resetAngle = gamepad1.y;

        //Lift
        lastLiftPos = liftPos;
        toggleAllianceHub = gamepad1.right_trigger > 0.2;
        toggleSharedHub = gamepad1.left_trigger > 0.2;
        toggleCapping = capButton.b.get();
        switchAllianceLevel = gamepad1.left_bumper;
        resetLift = gamepad1.back;

        //Intake
        freightInIntake = intake.getFreightInIntake();
        stopIntake = gamepad1.a;
        if (stopIntake && !lastStopIntake) killSwitch = !killSwitch;

        outtake = gamepad1.right_bumper;
        in = !killSwitch;

        //Spinner
        spin = spinButton.b.get();
    }

    @Override
    protected void updateStateMachine() {
        //Headless
        lastLeftX = leftX; lastLeftY = leftY; lastRightX = rightX;

        //Lift
        lastToggleAllianceHub = toggleAllianceHub;
        lastToggleSharedHub = toggleSharedHub;
        lastToggleCapping = toggleCapping;
        lastSwitchAllianceLevel = switchAllianceLevel;
        lastResetLift = resetLift;

        //Intake
        lastFreightInIntake = freightInIntake;
        lastOuttake = outtake;
        lastIn = in;
        lastStopIntake = stopIntake;

        //Spinner
        lastSpin = spin;
    }
}