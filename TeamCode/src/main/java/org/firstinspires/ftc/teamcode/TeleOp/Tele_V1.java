package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.CarouselSpinner;
import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.TurningIntake;

@TeleOp (name = "Tele_V1")
//@Disabled
public class Tele_V1 extends TeleOp_Base {

    //Headless
    private boolean resetAngle;

    private TurningIntake turningIntake;
    private Lift lift;
    private CarouselSpinner spinner;

    @Override
    public void init() {
        initializeDrive();
        initializeBulkRead();
        initializeGyro();
        initializeVars();

        turningIntake = new TurningIntake(hardwareMap, "intake", "wrist");
        lift = new Lift(hardwareMap, "lift", bReadEH);
        spinner = new CarouselSpinner(hardwareMap, "leftSpinner", "rightSpinner");
    }

    @Override
    public void loop() {
        updateBulkRead();
        gyro.update();
        getInput();

        // Drive
        driveAccelHeadless(gyro.getRawYaw(), resetAngle, 250, .6, gamepad1.a);

        // Turning Intake
        if (gamepad2.dpad_up) turningIntake.setWristCenter();
        else if (gamepad2.dpad_left) turningIntake.moveWristLeft();
        else if (gamepad2.dpad_right) turningIntake.moveWristRight();

        if (gamepad2.right_trigger > 0.1 || gamepad2.left_trigger > 0.1)
            turningIntake.setIntakePower(gamepad2.right_trigger - gamepad2.left_trigger);
        else turningIntake.setIntakePower(0);

        // Lift
        if (gamepad1.right_trigger > 0.1 || gamepad1.left_trigger > 0.1)
            lift.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        else
            lift.setPower(0);

        if (lift.getLiftHeight() > 10) lim = 0.5;
        else lim = 1;

        // Carousel
        spinner.setLeftPower(gamepad1.x ? 1 : 0);
        spinner.setRightPower(gamepad1.b ? 1 : 0);

        // Telemetry
        telemetry.addData("Lift Height: ", lift.getLiftHeight());

        turningIntake.update();
        lift.update();
        spinner.update();
        updateStateMachine();
    }

    @Override
    public void stop() {
        setMotorPowers(0,0,0,0);
        turningIntake.stop();
        lift.stop();
        spinner.stop();
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
