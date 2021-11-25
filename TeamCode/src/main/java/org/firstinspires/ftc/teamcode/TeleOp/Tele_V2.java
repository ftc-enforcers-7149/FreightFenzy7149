package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.CarouselSpinner;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.TurningIntake;

@TeleOp (name = "Tele_V2")
//@Disabled
public class Tele_V2 extends TeleOp_Base {

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

    @Override
    public void init() {
        initializeDrive();
        initializeBulkRead();
        initializeGyro();
        initializeVars();

        intake = new Intake(hardwareMap, "intake");//, "intakeColor");
        lift = new Lift(hardwareMap, "lift", bReadEH, false);
        spinner = new CarouselSpinner(hardwareMap, "leftSpinner", "rightSpinner");
    }

    @Override
    public void loop() {
        updateBulkRead();
        gyro.update();
        getInput();

        // Drive
        driveHeadless(gyro.getRawYaw(), resetAngle);

        //Intake
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

        if (gamepad1.back) lift.setManualOverride(true);

        // Carousel
        spinner.setLeftPower(gamepad1.x ? 1 : 0);
        spinner.setRightPower(gamepad1.b ? 1 : 0);

        // Telemetry
        telemetry.addData("Lift Height: ", lift.getLiftHeight());
        telemetry.addData("Freight in Intake: ", intake.getFreightInIntake());

        intake.update();
        lift.update();
        spinner.update();
        updateStateMachine();
    }

    @Override
    public void stop() {
        setMotorPowers(0,0,0,0);
        intake.stop();
        lift.stop();
        spinner.stop();
    }

    @Override
    protected void getInput() {
        //Headless
        leftX = curveInput(gamepad1.left_stick_x, 5)*lim * 0.9;
        leftY = curveInput(gamepad1.left_stick_y, 5)*lim * 0.9;
        rightX = curveInput(gamepad1.right_stick_x, 5)*lim*0.75 * 0.8;
        resetAngle = gamepad1.y;

        if (gamepad1.right_trigger > 0.1 || gamepad1.left_trigger > 0.1)
            liftPower = gamepad1.right_trigger - gamepad1.left_trigger;
        else
            liftPower = 0;

        if (gamepad1.dpad_up) liftPos = LiftPosition.HIGH;
        else if (gamepad1.dpad_left) liftPos = LiftPosition.MIDDLE;
        else if (gamepad1.dpad_right) liftPos = LiftPosition.LOW;
        else if (gamepad1.dpad_down) liftPos = LiftPosition.GROUND;
    }

    @Override
    protected void updateStateMachine() {
        lastLeftX = leftX; lastLeftY = leftY; lastRightX = rightX;
        lastLiftPower = liftPower;
        lastLiftPos = liftPos;
    }
}
