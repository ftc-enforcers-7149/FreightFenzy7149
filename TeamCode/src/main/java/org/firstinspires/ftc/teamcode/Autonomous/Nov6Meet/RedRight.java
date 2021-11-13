package org.firstinspires.ftc.teamcode.Autonomous.Nov6Meet;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.Autonomous_Base;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.CarouselSpinner;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.TurningIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam.OpenCV;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam.TSEPipeline;
import org.opencv.core.RotatedRect;

@Autonomous(name = "Red Right")
@Disabled
public class RedRight extends Autonomous_Base {

    private Intake intake;
    private Lift lift;
    private CarouselSpinner spinner;

    private OpenCV tseDetector;

    FtcDashboard dashboard;

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

        dashboard = FtcDashboard.getInstance();

        intake = new Intake(hardwareMap, "intake");
        lift = new Lift(hardwareMap, "lift", bReadEH);
        spinner = new CarouselSpinner(hardwareMap, "leftSpinner", "rightSpinner");

        tseDetector = new OpenCV(hardwareMap, dashboard);
        tseDetector.start(new TSEPipeline(320, 180, 320, 180));

        /// Init Loop ///

        while (!isStarted() && !isStopRequested()) {
            tseDetector.update();
            telemetry.addData("Hub Level: ", detectBarcode());
            telemetry.update();
        }
        if (isStopRequested()) return;

        /// Start ///

        HubLevel liftHeight = detectBarcode();
        tseDetector.stop();

        /// Loop ///

        //Set lift to correct level according to the vision
        switch (liftHeight) {
            case LOW:
                setLiftHeight(Lift.LOW_HEIGHT);
            case MIDDLE:
                setLiftHeight(Lift.MIDDLE_HEIGHT);
            case HIGH:
                setLiftHeight(Lift.HIGH_HEIGHT);
        }

        //Drive to hub
        driveTo(16,0,0);

        //Deliver pre-loaded block
        outtake();

        //Move back a little so that the intake doesn't hit the hub
        driveTo(10,0,0);

        //Put lift back down
        lift.setTargetHeight(Lift.GROUND_HEIGHT);

        //Realign with the wall and turn towards the warehouse
        driveTo(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), Math.toRadians(270));
        driveTo(0,0, Math.toRadians(270));
        setLiftHeight(Lift.GROUND_HEIGHT);

        //Start intaking
        intake.setIntakePower(1);

        //Drive into the warehouse
        driveTo(0,-47, Math.toRadians(270));

        //Stop intake
        intake.setIntakePower(0);

        //Drive backwards to the hub
        driveTo(0,0, Math.toRadians(270));

        //Turn and move towards the hub
        driveTo(10,0,Math.toRadians(270));
        lift.setTargetHeight(Lift.HIGH_HEIGHT);
        driveTo(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), 0);

        //Set lift to the highest height
        setLiftHeight(Lift.HIGH_HEIGHT);

        //Move forward
        driveTo(16,0,0);

        //Outtake the game element
        outtake();

        //Drive a little back and turn
        driveTo(10,0, 0);

        //Put lift back down
        lift.setTargetHeight(Lift.GROUND_HEIGHT);

        //Realign with the wall and turn towards the warehouse
        driveTo(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), Math.toRadians(270));

        //Park in warehouse
        driveTo(0,0, Math.toRadians(270));
        setLiftHeight(Lift.GROUND_HEIGHT);
        driveTo(0,47, Math.toRadians(270));

        /// Stop ///

        intake.stop();
        lift.stop();
        spinner.stop();
        setMotorPowers(0, 0, 0, 0);

        waitForTime(500);
    }

    private void outtake() {
        intake.setIntakePower(-1);
        waitForTime(1500);
        intake.setIntakePower(0);
    }

    private HubLevel detectBarcode() {
        RotatedRect boundingRect = tseDetector.getRect();
        if (boundingRect == null) return HubLevel.HIGH;
        if (boundingRect.center.x >= 3 * 640 / 4.0) {
            return HubLevel.MIDDLE;
        }
        else {
            return HubLevel.LOW;
        }
    }

    private void setLiftHeight(double height) {
        lift.setTargetHeight(height);
        while (opModeIsActive() && lift.getLiftHeight() < height - 0.5) {
            updateInputs();
            updateOutputs();
        }
    }

    @Override
    protected void subsystemUpdates() {
        intake.update();
        lift.update();
        spinner.update();
    }

    @Override
    protected void addTelemetryData() {
        telemetry.addData("Position: ", drive.getPoseEstimate());
        telemetry.addData("Lift Height: ", lift.getLiftHeight());
    }
}