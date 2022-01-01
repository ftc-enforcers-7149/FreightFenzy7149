package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.CarouselSpinner;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.LED.LED;

import static org.firstinspires.ftc.teamcode.GlobalData.HEADING;
import static org.firstinspires.ftc.teamcode.GlobalData.RAN_AUTO;

@TeleOp (name = "RED Tele_V2")
//@Disabled
public class Tele_V2_RED extends TeleOp_Base {
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
        GROUND, LOW, MIDDLE, HIGH;
    }
    private LiftPosition liftPos, lastLiftPos;

    private boolean resetLift, lastResetLift;
    private boolean manualOverride;

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

        // Intake
        if (gamepad2.right_trigger > 0.1 || gamepad2.left_trigger > 0.1)
            intake.setIntakePower(gamepad2.right_trigger - gamepad2.left_trigger);
        else intake.setIntakePower(0);

        // Lift
        if (liftPower != lastLiftPower)
            lift.setPower(liftPower);
        else if (liftPos != lastLiftPos) {
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
        spinner.setLeftPower(gamepad1.x ? 1 : 0);
        spinner.setRightPower(gamepad1.b ? -1 : 0);

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
        ledDisable();

        RAN_AUTO = false;
    }

    @Override
    protected void getInput() {
        //Headless
        leftX = curveInput(gamepad1.left_stick_x, 1)*lim * 0.75;
        leftY = curveInput(gamepad1.left_stick_y, 1)*lim * 0.75;
        rightX = curveInput(gamepad1.right_stick_x, 1)*lim*0.75 * 0.75;
        resetAngle = gamepad1.y;

        if (gamepad1.right_trigger > 0.1 || gamepad1.left_trigger > 0.1)
            liftPower = gamepad1.right_trigger - gamepad1.left_trigger;
        else
            liftPower = 0;

        if (gamepad2.dpad_up) liftPos = LiftPosition.HIGH;
        else if (gamepad2.dpad_left) liftPos = LiftPosition.MIDDLE;
        else if (gamepad2.dpad_right) liftPos = LiftPosition.LOW;
        else if (gamepad2.dpad_down) liftPos = LiftPosition.GROUND;

        if (gamepad1.dpad_up) liftPos = LiftPosition.HIGH;
        else if (gamepad1.dpad_left) liftPos = LiftPosition.MIDDLE;
        else if (gamepad1.dpad_right) liftPos = LiftPosition.LOW;
        else if (gamepad1.dpad_down) liftPos = LiftPosition.GROUND;

        resetLift = gamepad1.back;
    }

    @Override
    protected void updateStateMachine() {
        lastLeftX = leftX; lastLeftY = leftY; lastRightX = rightX;

        lastLiftPower = liftPower;
        lastLiftPos = liftPos;

        lastResetLift = resetLift;
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
