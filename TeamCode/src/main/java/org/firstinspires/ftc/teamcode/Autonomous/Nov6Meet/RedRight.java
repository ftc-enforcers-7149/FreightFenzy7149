package org.firstinspires.ftc.teamcode.Autonomous.Nov6Meet;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Autonomous_Base;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.CarouselSpinner;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.TurningIntake;

@Autonomous(name = "Red Right")
//@Disabled
public class RedRight extends Autonomous_Base {

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

        turningIntake = new TurningIntake(hardwareMap, "intake", "wrist", false);
        lift = new Lift(hardwareMap, "lift", bReadEH);
        spinner = new CarouselSpinner(hardwareMap, "leftSpinner", "rightSpinner");

        /// Start ///

        waitForStart();
        if (isStopRequested()) return;

        /// Loop ///

        //Spit out preloaded block
        turningIntake.setWristLeft();
        waitForTime(750);
        outtake();

        POS_ACC = 1;

        //Cycle "n" times
        for (int n = 0; n < 3; n++) {
            //Drive into warehouse while intaking
            turningIntake.setWristCenter();
            turningIntake.setIntakePower(1);
            waitForTime(750);
            driveTo(40 + n * 4, 0, 0);
            waitForTime(500);

            //Stop intaking and back out of warehouse
            turningIntake.setIntakePower(-0.1);
            waitForTime(300);
            turningIntake.setIntakePower(0);
            driveTo(0, 0, 0);

            //Outtake collected block
            turningIntake.setWristLeft();
            waitForTime(750);
            outtake();
        }

        //Park
        turningIntake.setWristCenter();
        waitForTime(750);
        POS_ACC = 0.1;
        driveTo(48, 0, 0);

        /// Stop ///

        setMotorPowers(0, 0, 0, 0);

        turningIntake.stop();
        lift.stop();
        spinner.stop();

        waitForTime(1000);
    }

    private void outtake() {
        turningIntake.setIntakePower(-1);

        double startTime = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() < startTime + 1500) {
            updateBulkRead();
            gyro.update();
            drive.update();

            updateSubsystems();
            updateTelemetry();
        }

        turningIntake.setIntakePower(0);
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