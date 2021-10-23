package org.firstinspires.ftc.teamcode.Autonomous.Oct23Meet;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Autonomous_Base;
import org.firstinspires.ftc.teamcode.Subsystems.CarouselSpinner;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.TurningIntake;

@Autonomous(name = "Blue Right")
public class BlueRight extends Autonomous_Base {

    private TurningIntake turningIntake;
    private Lift lift;
    private CarouselSpinner spinner;

    @Override
    public void runOpMode() throws InterruptedException {
        /// Init ///
        USE_SUBS = true;

        initializeDrive();
        initializeBulkRead();
        initializeGyro();
        try {
            initializeOdometry();
        } catch (Exception e) {
            e.printStackTrace();
            throw new InterruptedException();
        }

        turningIntake = new TurningIntake(hardwareMap, "intake", "wrist");
        lift = new Lift(hardwareMap, "lift", bReadEH);
        spinner = new CarouselSpinner(hardwareMap, "leftSpinner", "rightSpinner");

        /// Start ///

        waitForStart();
        if (isStopRequested()) return;

        /// Loop ///

        POS_ACC = 1;
        driveTo(4, -5, 0);

        double startTime = System.currentTimeMillis();
        turningIntake.setWristRight();
        while (opModeIsActive() && System.currentTimeMillis() < startTime + 4000) {
            updateBulkRead();
            gyro.update();
            drive.update();

            spinner.setRightPower(0.75);
            turningIntake.setIntakePower(-1);

            updateSubsystems();
            updateTelemetry();
        }
        spinner.setRightPower(0);
        turningIntake.setWristLeft();
        turningIntake.setIntakePower(0);

        POS_ACC = 0.1;
        driveTo(30, -5, 0);

        /// Stop ///

        setMotorPowers(0, 0, 0, 0);

        turningIntake.stop();
        lift.stop();
        spinner.stop();

        startTime = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() < startTime + 1000);
    }

    @Override
    protected void subsystemUpdates() {
        turningIntake.update();
        lift.update();
        spinner.update();
    }

    @Override
    protected void addTelemetryData() {
        telemetry.addData("Position: ", drive.getPoseEstimate());
        telemetry.addData("Lift Height: ", lift.getLiftHeight());
    }
}