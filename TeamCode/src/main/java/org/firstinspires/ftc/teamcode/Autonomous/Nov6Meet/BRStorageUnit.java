package org.firstinspires.ftc.teamcode.Autonomous.Nov6Meet;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.Autonomous_Base;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.CarouselSpinner;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.TurningIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam.OpenCV;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam.TSEPipeline;
import org.opencv.core.RotatedRect;

@Autonomous(name = "Blue Right Storage Unit")
@Disabled
public class BRStorageUnit extends Autonomous_Base {

    private TurningIntake turningIntake;
    private Lift lift;
    private CarouselSpinner spinner;

    private OpenCV tseDetector;

    //FtcDashboard dashboard;

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

        //dashboard = FtcDashboard.getInstance();

        turningIntake = new TurningIntake(hardwareMap, "intake", "wrist", false);
        lift = new Lift(hardwareMap, "lift", bReadEH);
        spinner = new CarouselSpinner(hardwareMap, "leftSpinner", "rightSpinner");

        tseDetector = new OpenCV(hardwareMap);//, dashboard);
        tseDetector.start(new TSEPipeline(0, 180, 320, 180));

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

        //Drive to the duckwheel
        driveTo(5, -4, 0);

        //Spin and stop duckwheel
        spinner.setRightPower(0.75);
        waitForTime(4000);
        spinner.setRightPower(0);

        //Drive to hub
        driveTo(34,26, Math.toRadians(60));

        //Set lift to correct level according to the vision
        switch (liftHeight) {
            case LOW:
                setLiftHeight(Lift.LOW_HEIGHT);
                break;
            case MIDDLE:
                setLiftHeight(Lift.MIDDLE_HEIGHT);
                break;
            case HIGH:
                setLiftHeight(Lift.HIGH_HEIGHT);
                break;
        }

        //Drive to hub and outtake
        driveTo(36,30, Math.toRadians(60));
        outtake();

        //Drive a little bit back and drop lift
        driveTo(26,26, Math.toRadians(60));
        lift.setTargetHeight(Lift.GROUND_HEIGHT);

        //Align with the warehouse and park
        driveTo(26,20, Math.toRadians(90));
        turningIntake.setWristRight();
        driveTo(26,-12, Math.toRadians(90));

        //Lower lift all the way down for TeleOp
        setLiftHeight(Lift.GROUND_HEIGHT);

        /// Stop ///

        turningIntake.stop();
        lift.stop();
        spinner.stop();
        setMotorPowers(0, 0, 0, 0);

        waitForTime(1000);
    }

    private void outtake() {
        turningIntake.setIntakePower(-1);
        waitForTime(1500);
        turningIntake.setIntakePower(0);
    }

    private HubLevel detectBarcode() {
        RotatedRect boundingRect = tseDetector.getRect();
        if (boundingRect == null) return HubLevel.LOW;
        if (boundingRect.center.x <= 640 / 4.0) {
            return HubLevel.MIDDLE;
        }
        else {
            return HubLevel.HIGH;
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