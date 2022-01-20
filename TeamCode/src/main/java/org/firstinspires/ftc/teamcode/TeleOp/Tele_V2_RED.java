package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.CarouselSpinner;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.MotorIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Levels;

import static org.firstinspires.ftc.teamcode.GlobalData.HEADING;
import static org.firstinspires.ftc.teamcode.GlobalData.RAN_AUTO;

@TeleOp (name = "RED Tele_V2")
//@Disabled
public class Tele_V2_RED extends TeleOp_Base {

    //Headless
    private boolean resetAngle;

    //Control objects
    private MotorIntake intake;
    private Lift lift;
    private CarouselSpinner spinner;

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
    private double outtake, lastOuttake, in, lastIn;
    private boolean stopIntake, lastStopIntake;
    private boolean killSwitch;

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
                "intake", "paddle", "latch");
                //"intakeColor");
        lift = new Lift(hardwareMap, "lift", bReadEH, !RAN_AUTO);
        spinner = new CarouselSpinner(hardwareMap, "leftSpinner", "rightSpinner");

        if (RAN_AUTO) gyro.setOffset(HEADING);
        RAN_AUTO = false;

        addInput(intake);
        addInput(lift);
        addOutput(intake);
        addOutput(lift);
        addOutput(spinner);
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

        if (lift.getHeight() > 10) {
            lim = 0.6;
        }
        else {
            lim = 1;
        }

        //Lift

        //Change height
        if (switchAllianceLevel && !lastSwitchAllianceLevel) {
            if (liftPos == Levels.CAP)
                capDown = !capDown;
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
            if (capDown)
                liftPos = Levels.HIGH;
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

        //Intake
        if (stopIntake && !lastStopIntake) killSwitch = !killSwitch;

        if (in > 0.1) {
            intake.setIntakePower(in);
            intake.setPaddle(MotorIntake.PaddlePosition.BACK);
            intake.setLatch(MotorIntake.LatchPosition.OPEN);
        }
        else if (lastIn > 0.1) {
            intake.setIntakePower(0);
            intake.setPaddle(MotorIntake.PaddlePosition.BACK);
            intake.setLatch(MotorIntake.LatchPosition.CLOSED);
        }
        else if (outtake > 0.1) {
            intake.setIntakePower(-outtake);
            intake.setPaddle(MotorIntake.PaddlePosition.OUT);
            intake.setLatch(MotorIntake.LatchPosition.OPEN);
        }
        else if (lastOuttake > 0.1) {
            intake.setIntakePower(0);
            intake.setPaddle(MotorIntake.PaddlePosition.BACK);
            intake.setLatch(MotorIntake.LatchPosition.OPEN);
        }
        else {
            intake.setIntakePower(0);
        }
        /*else if (killSwitch)
            intake.setIntakePower(0);
        else if (freightInIntake)
            intake.setIntakePower(-0.2);
        else
            intake.setIntakePower(-1);*/

        // Carousel
        spinner.setLeftPower(spin ? 1 : 0);
        spinner.setRightPower(spin ? -1 : 0);

        // Telemetry
        telemetry.addData("Lift Height: ", lift.getHeight());
        telemetry.addData("Freight in Intake: ", intake.getFreightInIntake());

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
        toggleCapping = gamepad1.b;
        switchAllianceLevel = gamepad1.left_bumper;
        resetLift = gamepad1.back;

        //Intake
        freightInIntake = intake.getFreightInIntake();
        outtake = gamepad2.left_trigger;
        in = gamepad2.right_trigger;
        stopIntake = gamepad2.a;

        //Spinner
        spin = gamepad1.x;
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
