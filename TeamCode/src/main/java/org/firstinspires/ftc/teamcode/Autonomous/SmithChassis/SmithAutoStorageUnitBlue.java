package org.firstinspires.ftc.teamcode.Autonomous.SmithChassis;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.Autonomous_Base;

@Autonomous(name = "Smith Auto Blue Storage Unit")
@Disabled
public class SmithAutoStorageUnitBlue extends Autonomous_Base {

    @Override
    public void runOpMode() throws InterruptedException {
        /// Init ///

        initializeDrive();
        initializeGyro();

        /// Start ///

        waitForStart();
        if (isStopRequested()) return;

        /// Loop ///

        tankDriveTo(-0.50,-0.50,680);

        /// Stop ///

        setMotorPowers(0, 0, 0, 0);
    }

    @Override
    protected void subsystemUpdates() {

    }

    protected void tankDriveTo(double powerMotorLeft, double powerMotorRight, double time) {
        // beginning of the function
        double startTime = System.currentTimeMillis();

        while (System.currentTimeMillis() <= startTime + time) {
            tankDriveMotorPowers(powerMotorLeft, powerMotorRight);
        }

        setMotorPowers(0, 0, 0, 0);
    }

    protected void tankDriveMotorPowers(double powerLeftMotors, double powerRightMotors) {
        setMotorPowers(powerLeftMotors, powerRightMotors, powerLeftMotors, powerRightMotors);
    }

    @Override
    protected void addTelemetryData() {

    }
}
