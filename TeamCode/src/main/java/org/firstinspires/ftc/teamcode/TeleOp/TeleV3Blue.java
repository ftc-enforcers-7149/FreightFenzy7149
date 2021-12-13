package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.CarouselSpinner;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Turret;

@TeleOp (name = "Tele V3 Blue")
//@Disabled
public class TeleV3Blue extends TeleOp_Base {

    //Headless
    private boolean resetAngle;

    private CarouselSpinner spinner;
    private Intake intake;
    private Elevator elevator;
    private Turret turret;

    //Intake control
    private boolean forward, lastForward, backward, lastBackward;

    //Elevator control
    private double liftPower, lastLiftPower;
    private Elevator.Level level, lastLevel;
    private boolean resetLift, lastResetLift;
    private boolean manualOverride;

    //Turret control
    private double angle, lastAngle;

    @Override
    public void init() {
        //initializeAll();
        initializeWithoutDrive();

        spinner = new CarouselSpinner(hardwareMap, "spinner");
        intake = new Intake(hardwareMap, "intake", "intakeColor", "outtakeColor");
        elevator = new Elevator(hardwareMap, "elevator", bReadEH);
        turret = new Turret(hardwareMap, "turret", bReadEH);

        addInput(intake);
        addInput(elevator);
        addInput(turret);

        addOutput(spinner);
        addOutput(intake);
        addOutput(elevator);
        addOutput(turret);

        lastForward = false; lastBackward = false;
        lastLiftPower = 0;
        level = Elevator.Level.GROUND;
        lastLevel = Elevator.Level.GROUND;
        lastResetLift = false;
        manualOverride = false;
    }

    @Override
    public void loop() {
        updateInputs();
        getInput();

        // Drive
        driveHeadless(gyro.getYaw(), resetAngle);

        //Intake
        if (forward && !lastForward) intake.forward();
        else if (backward && !lastBackward) intake.backward();

        // Elevator
        if (liftPower != lastLiftPower)
            elevator.setPower(liftPower);
        else if (level != lastLevel) {
            elevator.setTargetHeight(level);
        }

        if (resetLift && !lastResetLift) {
            elevator.setManualOverride(!manualOverride);
            manualOverride = !manualOverride;
        }

        if (elevator.getHeight() > 10) {
            lim = 0.6;
        }
        else {
            lim = 1;
        }

        //Turret
        if (angle != lastAngle)
            turret.setTargetAngle(angle);

        // Carousel
        spinner.setPower(Alliance.BLUE, gamepad1.b?1:0);

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
        leftX = curveInput(gamepad1.left_stick_x, 2)*lim;
        leftY = curveInput(gamepad1.left_stick_y, 2)*lim;
        rightX = curveInput(gamepad1.right_stick_x, 2)*lim*0.75;
        resetAngle = gamepad1.y;

        //Intake
        forward = gamepad2.right_trigger >= 0.2;
        backward = gamepad2.left_trigger >= 0.2;

        //Elevator
        if (gamepad1.right_trigger > 0.1 || gamepad1.left_trigger > 0.1)
            liftPower = gamepad1.right_trigger - gamepad1.left_trigger;
        else
            liftPower = 0;

        if (gamepad2.dpad_up) level = Elevator.Level.HIGH;
        else if (gamepad2.dpad_left) level = Elevator.Level.MIDDLE;
        else if (gamepad2.dpad_right) level = Elevator.Level.LOW;
        else if (gamepad2.dpad_down) level = Elevator.Level.GROUND;

        if (gamepad1.dpad_up) level = Elevator.Level.HIGH;
        else if (gamepad1.dpad_left) level = Elevator.Level.MIDDLE;
        else if (gamepad1.dpad_right) level = Elevator.Level.LOW;
        else if (gamepad1.dpad_down) level = Elevator.Level.GROUND;

        resetLift = gamepad1.back;

        //Turret
        if (gamepad2.right_stick_y != 0 && gamepad2.right_stick_x != 0)
            angle = Math.atan2(-gamepad2.right_stick_y, gamepad2.right_stick_x) - Math.toRadians(gyro.getYaw());
    }

    @Override
    protected void updateStateMachine() {
        lastLeftX = leftX; lastLeftY = leftY; lastRightX = rightX;
        lastForward = forward; lastBackward = backward;
        lastLiftPower = liftPower;
        lastLevel = level;
        lastResetLift = resetLift;
        lastAngle = angle;
    }
}
