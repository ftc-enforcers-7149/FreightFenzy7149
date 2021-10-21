package org.firstinspires.ftc.teamcode.Autonomous.Oct23Meet;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Autonomous_Base;
import org.firstinspires.ftc.teamcode.Subsystems.CarouselSpinner;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.TurningIntake;

@Autonomous(name = "Red Left")
public class RedLeft extends Autonomous_Base {

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

        driveTo(4, 7, 0);

        double startTime = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() < startTime + 6000) {
            updateBulkRead();
            gyro.update();
            drive.update();

            spinner.setLeftPower(1);
            turningIntake.setIntakePower(-1);

            updateSubsystems();
            updateTelemetry();
        }
        spinner.setLeftPower(0);
        turningIntake.setIntakePower(0);

        driveTo(24, 8, 0);

        /// Stop ///

        setMotorPowers(0, 0, 0, 0);

        turningIntake.stop();
        lift.stop();
        spinner.stop();
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