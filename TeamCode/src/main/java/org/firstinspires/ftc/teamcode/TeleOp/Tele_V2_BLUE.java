package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.CarouselSpinner;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.LED.LED;

import static org.firstinspires.ftc.teamcode.GlobalData.HEADING;
import static org.firstinspires.ftc.teamcode.GlobalData.RAN_AUTO;

@TeleOp (name = "BLUE Tele_V2")
//@Disabled
public class Tele_V2_BLUE extends TeleOp_Base {

    //LED
    public LED led;
    private boolean ledEnabled; //true if LED is enabled

    //Headless
    private boolean resetAngle;

    private Intake intake;
    private Lift lift;
    private CarouselSpinner spinner;

    private double liftPower, lastLiftPower;

    private enum LiftPosition {
        GROUND, HUB, LOW, MIDDLE, HIGH;
    }
    private LiftPosition liftPos = LiftPosition.GROUND, liftToggle = LiftPosition.HIGH, lastLiftPos, lastLiftToggle;

    private boolean intakeBlock, lastIntakeBlock;
    private boolean resetLift, lastResetLift;
    private boolean manualOverride;
    private boolean toggleIntake, lastToggleIntake, toggleLift, lastToggleLift, toggleLiftUp, lastToggleLiftUp, liftUp;
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
        lastResetLift = false;
        manualOverride = false;

        ledEnable();
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
        ledUpdate();

        // Drive
        driveHeadless(gyro.getYaw(), resetAngle);

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

        if (intakeBlock != lastIntakeBlock) {

            intake.setIntakePower(intakeBlock ? (intake.getFreightInIntake() ? -0.2 : -1 ) : 1);

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
        spinner.setRightPower(gamepad1.b ? 1 : 0);

        // Telemetry
        telemetry.addData("Lift Height: ", lift.getLiftHeight());
        telemetry.addData("Freight in Intake: ", intake.getFreightInIntake());
        telemetry.addData("Intake dist: ", intake.distance.getValue());

        updateOutputs();
        updateStateMachine();
    }

    @Override
    public void stop() {
        stopInputs();
        stopOutputs();
        ledDisable();
    }

    @Override
    protected void getInput() {
        //Headless
        leftX = curveInput(gamepad1.left_stick_x, 1)*lim * 0.75;
        leftY = curveInput(gamepad1.left_stick_y, 1)*lim * 0.75;
        rightX = curveInput(gamepad1.right_stick_x, 1)*lim*0.75 * 0.75;

        freightInIntake = intake.getFreightInIntake();

        toggleIntake = gamepad1.right_bumper;
        if(toggleIntake && !lastToggleIntake) {

            intakeBlock = !intakeBlock;

        }

        toggleLift = gamepad1.left_bumper;
        if(toggleLift && !lastToggleLift) {

            switch(liftPos) {

                case GROUND:
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

        toggleLiftUp = gamepad1.right_trigger > 0.2;
        if(toggleLiftUp && !lastToggleLiftUp) {
            liftUp = !liftUp;
        }

        if(liftUp) liftPos = liftToggle;
        else liftPos = LiftPosition.GROUND;

        if(gamepad1.left_trigger > 0.2) liftPos = LiftPosition.LOW;

        resetAngle = gamepad1.y;
        resetLift = gamepad1.back;
    }

    @Override
    protected void updateStateMachine() {
        lastLeftX = leftX; lastLeftY = leftY; lastRightX = rightX;

        lastLiftPower = liftPower;
        lastLiftPos = liftPos;

        lastResetLift = resetLift;

        lastToggleIntake = toggleIntake;
        lastToggleLift = toggleLift;
        lastLiftToggle = liftToggle;
        lastToggleLiftUp = toggleLiftUp;
        lastIntakeBlock = intakeBlock;
    }

    /**
     * disable LEDs
     */
    public void ledDisable(){
        ledEnabled = false;
    }

    /**
     * enable LEDs
     */
    public void ledEnable(){
        ledEnabled = true;
    }


    /**
     * sets LEDs to value.
     * main lED code
     */
    public void ledUpdate() {
        if (intake.getFreightInIntake()) led.green();
        else led.red();
    }
}
