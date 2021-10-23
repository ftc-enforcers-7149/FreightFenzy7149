package org.firstinspires.ftc.teamcode.Autonomous.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.Autonomous_Base;

import static org.firstinspires.ftc.teamcode.Subsystems.FixedRoadrunner.createVector2d;

@Autonomous(name = "Simple Square")
@Disabled
public class SimpleSquare extends Autonomous_Base {

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

        updateBulkRead();
        gyro.update();
        drive.update();
        updateSubsystems();
        updateTelemetry();

        //Drive forward
        drive.followTrajectoryAsync(
                drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToConstantHeading(createVector2d(0, 20))
                        .build()
        );

        while (opModeIsActive() && drive.isBusy()) {
            updateBulkRead();
            gyro.update();
            drive.update();
            updateSubsystems();
            updateTelemetry();
        }

        //Rotate
        drive.turnAsync(Math.toRadians(180));

        while (opModeIsActive() && drive.isBusy()) {
            updateBulkRead();
            drive.update();
            updateSubsystems();
            updateTelemetry();
        }

        //Drive right
        drive.followTrajectoryAsync(
                drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToConstantHeading(createVector2d(20, 20))
                        .build()
        );

        while (opModeIsActive() && drive.isBusy()) {
            updateBulkRead();
            gyro.update();
            drive.update();
            updateSubsystems();
            updateTelemetry();
        }

        //Rotate
        drive.turnAsync(-Math.toRadians(180));

        while (opModeIsActive() && drive.isBusy()) {
            updateBulkRead();
            gyro.update();
            drive.update();
            updateSubsystems();
            updateTelemetry();
        }

        //Drive backward
        drive.followTrajectoryAsync(
                drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToConstantHeading(createVector2d(20, 0))
                        .build()
        );

        while (opModeIsActive() && drive.isBusy()) {
            updateBulkRead();
            drive.update();
            updateSubsystems();
            updateTelemetry();
        }

        //Rotate
        drive.turnAsync(Math.toRadians(180));

        while (opModeIsActive() && drive.isBusy()) {
            updateBulkRead();
            gyro.update();
            drive.update();
            updateSubsystems();
            updateTelemetry();
        }

        //Drive left
        drive.followTrajectoryAsync(
                drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToConstantHeading(createVector2d(0, 0))
                        .build()
        );

        while (opModeIsActive() && drive.isBusy()) {
            updateBulkRead();
            gyro.update();
            drive.update();
            updateSubsystems();
            updateTelemetry();
        }

        //Rotate
        drive.turnAsync(-Math.toRadians(180));

        while (opModeIsActive() && drive.isBusy()) {
            updateBulkRead();
            gyro.update();
            drive.update();
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
