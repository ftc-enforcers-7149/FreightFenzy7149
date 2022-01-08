package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.CarouselSpinner;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Lift;

import static org.firstinspires.ftc.teamcode.GlobalData.HEADING;
import static org.firstinspires.ftc.teamcode.GlobalData.RAN_AUTO;

@TeleOp (name = "BLUE Tele_V2 OLD")
@Disabled
public class Tele_V2_BLUE extends TeleOp_Base {

    //Headless
    private boolean resetAngle;

    private Intake intake;
    private Lift lift;
    private CarouselSpinner spinner;

    private double liftPower, lastLiftPower;

    private enum LiftPosition {
        GROUND, HUB, LOW, MIDDLE, HIGH;
    }
    private LiftPosition liftPos, liftToggle, lastLiftPos;

    private boolean intakeBlock, lastIntakeBlock;
    private boolean resetLift, lastResetLift;
    private boolean manualOverride;
    private boolean toggleIntake, lastToggleIntake, toggleLift, lastToggleLift, toggleLiftUp, lastToggleLiftUp, liftUp, toggleLiftLow, lastToggleLiftLow, liftLow, lastLiftUp, lastLiftLow;
    private boolean killswitch, toggleKillswitch, lastToggleKillswitch;
    private boolean freightInIntake;

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

        lastLiftPower = 0;
        liftPos = LiftPosition.GROUND;
        lastLiftPos = LiftPosition.GROUND;
        liftToggle = LiftPosition.HIGH;
        lastResetLift = false;
        lastToggleLift = false;
        lastIntakeBlock = false;
        lastToggleKillswitch = false;
        lastToggleLiftUp = false;
        manualOverride = false;
        intakeBlock = true;
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

        if(toggleKillswitch && !lastToggleKillswitch) {
            killswitch = !killswitch;
        }

        if(toggleIntake) {
            intakeBlock = false;
        }
        else {
            intakeBlock = true;
        }
        /*if(!freightInIntake) intakeBlock = true*/;

        // Drive
        driveHeadless(gyro.getYaw(), resetAngle);

        if(toggleLift && !lastToggleLift) {

            switch(liftToggle) {
                case LOW:
                    liftToggle = LiftPosition.HIGH;
                    break;
                case MIDDLE:
                    liftToggle = LiftPosition.LOW;
                    break;
                case HIGH:
                    liftToggle = LiftPosition.MIDDLE;
                    break;

            }

        }

        if(toggleLiftUp && !lastToggleLiftUp) {
            liftUp = !liftUp;
        }

        if(toggleLiftLow && !lastToggleLiftLow) liftLow = !liftLow;

        if(liftUp && !liftLow) {
            liftPos = liftToggle;
        }
        else if(!liftUp && liftLow) {
            liftPos = LiftPosition.LOW;
        }
        else if(liftUp && liftLow && !lastLiftUp){
            liftPos = liftToggle;
            liftLow = false;
        }
        else if(liftUp && liftLow && !lastLiftLow){
            liftPos = LiftPosition.LOW;
            liftUp = false;
        }
        else {
            liftPos = LiftPosition.GROUND;
        }

        if (liftPos != lastLiftPos) {
            switch (liftPos) {
                case HIGH:
                    lift.setTargetHeight(Lift.HIGH_HEIGHT);
                    break;
                case MIDDLE:
                    lift.setTargetHeight(Lift.MIDDLE_HEIGHT);
                    break;
                case LOW:
                    lift.setTargetHeight(Lift.LOW_HEIGHT);
                    break;
                case GROUND:
                    lift.setTargetHeight(0);
                    break;
            }
        }

        if (!killswitch) {

            intake.setIntakePower(intakeBlock ? (freightInIntake ? -0.2 : -1) : 1);

        }
        else {
            intake.setIntakePower(0);
        }

        if (resetLift && !lastResetLift) {
            lift.setManualOverride(!manualOverride);
            manualOverride = !manualOverride;
        }

        if (lift.getLiftHeight() > 10) {
            lim = 0.6;
        }
        else {
            lim = 1;
        }

        // Carousel
        spinner.setLeftPower(gamepad1.x ? -1 : 0);
        spinner.setRightPower(gamepad1.x ? -1 : 0);

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

        freightInIntake = intake.getFreightInIntake();
        toggleIntake = gamepad1.right_bumper;

        toggleLift = gamepad1.left_bumper;
        toggleLiftUp = gamepad1.right_trigger > 0.1;
        toggleLiftLow = gamepad1.left_trigger > 0.1;


        resetAngle = gamepad1.y;
        resetLift = gamepad1.back;

        toggleKillswitch = gamepad1.a;

    }

    @Override
    protected void updateStateMachine() {
        lastLeftX = leftX; lastLeftY = leftY; lastRightX = rightX;

        lastLiftPower = liftPower;
        lastLiftPos = liftPos;

        lastResetLift = resetLift;

        lastToggleIntake = toggleIntake;
        lastToggleLift = toggleLift;
        lastToggleLiftUp = toggleLiftUp;
        lastIntakeBlock = intakeBlock;
        lastToggleKillswitch = toggleKillswitch;
        lastToggleLiftLow = toggleLiftLow;
        lastLiftUp = liftUp;
        lastLiftLow = liftLow;
    }
}
