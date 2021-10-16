package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Test Drive To")
@Disabled
public class TestDriveTo extends Autonomous_Base {

    @Override
    public void runOpMode() throws InterruptedException {
        /// Init ///

        initializeDrive();
        initializeBulkRead();
        initializeGyro();
        try {
            initializeOdometry();
        } catch (Exception e) {
            e.printStackTrace();
            throw new InterruptedException();
        }

        /// Start ///

        waitForStart();
        if (isStopRequested()) return;

        /// Loop ///

        while (opModeIsActive()) {

            updateBulkRead();
            gyro.update();
            drive.update();

            driveTo(24, 24, Math.PI / 2);
            driveTo(0, 0, 0);

            updateSubsystems();
            updateTelemetry();
        }

        /// Stop ///

        setMotorPowers(0, 0, 0, 0);
    }

    @Override
    protected void subsystemUpdates() {

    }

    @Override
    protected void addTelemetryData() {
        telemetry.addData("Position", drive.getPoseEstimate());
    }
}
