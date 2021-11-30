package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.CarouselSpinner;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Scorer;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Turret;

@TeleOp (name = "Tele_V2")
//@Disabled
public class Tele_V3 extends TeleOp_Base {

    //Headless
    private boolean resetAngle;

    private CarouselSpinner spinner;
    private Intake intake;
    private Elevator elevator;
    private Turret turret;
    private Scorer scorer;

    @Override
    public void init() {
        try {
            initializeAll();
        } catch (Exception e) {
            e.printStackTrace();
            requestOpModeStop();
            return;
        }

        spinner = new CarouselSpinner(hardwareMap, "spinner");
        intake = new Intake(hardwareMap, "intake", "intakeColor");
        elevator = new Elevator(hardwareMap, "elevator", bReadEH);
        turret = new Turret(hardwareMap, "turret", bReadEH);
        scorer = new Scorer(elevator, turret);

        addInput(intake);
        addInput(elevator);
        addInput(turret);

        addOutput(spinner);
        addOutput(intake);
        addOutput(scorer);
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
        driveHeadless(gyro.getRawYaw(), resetAngle);

        //Intake
        if (gamepad2.right_trigger > 0.1 || gamepad2.left_trigger > 0.1)
            intake.setIntakePower(gamepad2.right_trigger - gamepad2.left_trigger);
        else intake.setIntakePower(0);

        // Scorer


        // Carousel
        spinner.setPower((gamepad1.b?1:0)-(gamepad1.x?1:0));

        // Telemetry
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
        leftX = curveInput(gamepad1.left_stick_x, 5)*lim;
        leftY = curveInput(gamepad1.left_stick_y, 5)*lim;
        rightX = curveInput(gamepad1.right_stick_x, 5)*lim*0.75;
        resetAngle = gamepad1.y;
    }

    @Override
    protected void updateStateMachine() {
        lastLeftX = leftX; lastLeftY = leftY; lastRightX = rightX;
    }
}
