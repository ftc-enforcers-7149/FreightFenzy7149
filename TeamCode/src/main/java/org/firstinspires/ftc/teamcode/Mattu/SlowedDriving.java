package org.firstinspires.ftc.teamcode.Mattu;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.VelLimitsJerk;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOp_Base;

@TeleOp(name = "Slowed Driving")
@Disabled
public class SlowedDriving extends TeleOp_Base {

    //Headless
    protected Gyroscope gyro;
    private boolean resetAngle;

    private Lift lift;
    private Servo wrist;

    private VelLimitsJerk driveX, driveY, turn;

    @Override
    public void init() {
        initializeDrive();
        initializeBulkRead();
        initializeVars();

        gyro = new Gyroscope(hardwareMap);

        lift = new Lift(hardwareMap, "lift", bReadEH);

        wrist = hardwareMap.servo.get("wrist");
        wrist.setDirection(Servo.Direction.REVERSE);
        wrist.setPosition(0.45);

        driveX = new VelLimitsJerk(1, 1, 1);
        driveY = new VelLimitsJerk(1, 1, 1);
        turn = new VelLimitsJerk(1, 1, 1);
    }

    @Override
    public void loop() {
        updateBulkRead();
        getInput();

        // Drive
        driveHeadless(gyro.getNewRawYaw(), resetAngle);

        // Lift
        if (gamepad1.right_trigger > 0.1 || gamepad1.left_trigger > 0.1)
            lift.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        else
            lift.setPower(0);

        if (lift.getLiftHeight() > 5) {
            driveX.setMaxTime(2);
            driveY.setMaxTime(2);
            turn.setMaxTime(2);
        }
        else {
            driveX.setMaxTime(1);
            driveY.setMaxTime(1);
            turn.setMaxTime(1);
        }

        // Telemetry
        telemetry.addData("Lift Height: ", lift.getLiftHeight());

        lift.update();
        updateStateMachine();
    }

    @Override
    public void stop() {
        setMotorPowers(0, 0, 0, 0);
        lift.stop();
    }

    @Override
    protected void getInput() {
        //Headless
        leftX = driveX.update(gamepad1.left_stick_x);
        leftY = driveY.update(gamepad1.left_stick_y);
        rightX = turn.update(gamepad1.right_stick_x);
        resetAngle = gamepad1.y;
    }

    @Override
    protected void updateStateMachine() {
        lastLeftX = leftX; lastLeftY = leftY; lastRightX = rightX;
    }
}
