package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.CarouselSpinner;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Lift;

import static org.firstinspires.ftc.teamcode.GlobalData.HEADING;
import static org.firstinspires.ftc.teamcode.GlobalData.RAN_AUTO;

@TeleOp (name = "BLUE Tele_V2")
//@Disabled
public class Tele_V2_BLUE extends TeleOp_Base {

    //Headless
    private boolean resetAngle;

    //Control objects
    private Intake intake;
    private Lift lift;
    private CarouselSpinner spinner;

    //Lift
    private enum LiftPosition {
        GROUND(Lift.GROUND_HEIGHT),
        LOW(Lift.LOW_HEIGHT),
        MIDDLE(Lift.MIDDLE_HEIGHT),
        HIGH(Lift.HIGH_HEIGHT),
        CAP_DOWN(Lift.HIGH_HEIGHT - 2),
        CAP(Lift.MAX_HEIGHT);

        public double pos;

        LiftPosition(double pos) {
            this.pos = pos;
        }

        public LiftPosition cycleBackward() {
            switch (this) {
                case HIGH: return MIDDLE;
                case MIDDLE: return LOW;
                case LOW:
                default: return HIGH;
            }
        }
    }
    private LiftPosition liftPos = LiftPosition.GROUND, lastLiftPos = LiftPosition.GROUND;
    private LiftPosition allianceLevel = LiftPosition.HIGH;

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
    private boolean outtake, lastOuttake;
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

        intake = new Intake(hardwareMap, "intake", "intakeColor");
        lift = new Lift(hardwareMap, "lift", bReadEH, !RAN_AUTO);
        spinner = new CarouselSpinner(hardwareMap, "leftSpinner", "rightSpinner");

        if (RAN_AUTO) gyro.setOffset(HEADING);

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

        if (lift.getLiftHeight() > 10) {
            lim = 0.6;
        }
        else {
            lim = 1;
        }

        //Lift

        //Change height
        if (switchAllianceLevel && !lastSwitchAllianceLevel) {
            if (liftPos == LiftPosition.CAP)
                capDown = !capDown;
            else
                allianceLevel = allianceLevel.cycleBackward();
        }

        if (toggleAllianceHub && !lastToggleAllianceHub) {
            atAllianceLevel = (lastLiftPos != allianceLevel && lastLiftPos != LiftPosition.CAP
                                && lastLiftPos != LiftPosition.CAP_DOWN);
            atSharedLevel = false;
            capping = false;
        }
        else if (toggleSharedHub && !lastToggleSharedHub) {
            atAllianceLevel = false;
            atSharedLevel = (lastLiftPos != LiftPosition.LOW);
            capping = false;
        }
        else if (toggleCapping && !lastToggleCapping) {
            atAllianceLevel = false;
            atSharedLevel = false;
            capping = (lastLiftPos != LiftPosition.CAP);
        }

        if (capping) {
            if (capDown)
                liftPos = LiftPosition.CAP_DOWN;
            else
                liftPos = LiftPosition.CAP;
        }
        else {
            capDown = false;
            if (atAllianceLevel) liftPos = allianceLevel;
            else if (atSharedLevel) liftPos = LiftPosition.LOW;
            else liftPos = LiftPosition.GROUND;
        }

        //Set height
        if (liftPos != lastLiftPos) {
            lift.setTargetHeight(liftPos.pos);
        }

        //Reset
        if (resetLift && !lastResetLift) {
            lift.setManualOverride(!manualOverride);
            manualOverride = !manualOverride;
        }

        //Intake
        if (stopIntake && !lastStopIntake) killSwitch = !killSwitch;

        if (outtake)
            intake.setIntakePower(1);
        else if (killSwitch)
            intake.setIntakePower(0);
        else if (freightInIntake)
            intake.setIntakePower(-0.2);
        else
            intake.setIntakePower(-1);

        // Carousel
        spinner.setLeftPower(spin ? 1 : 0);
        spinner.setRightPower(spin ? -1 : 0);

        // Telemetry
        telemetry.addData("Lift Height: ", lift.getLiftHeight());
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
        outtake = gamepad1.right_bumper;
        stopIntake = gamepad1.a;

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
        lastStopIntake = stopIntake;

        //Spinner
        lastSpin = spin;
    }
}
