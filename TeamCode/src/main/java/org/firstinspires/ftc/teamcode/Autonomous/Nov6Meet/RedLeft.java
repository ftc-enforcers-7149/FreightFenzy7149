package org.firstinspires.ftc.teamcode.Autonomous.Nov6Meet;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Autonomous_Base;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.CarouselSpinner;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.TurningIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam.OpenCV;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam.TSEPipeline;
import org.opencv.core.RotatedRect;

@Autonomous(name = "Red Left")
//@Disabled
public class RedLeft extends Autonomous_Base {

    private Intake intake;
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

        intake = new Intake(hardwareMap, "intake");
        intake.update();
        lift = new Lift(hardwareMap, "lift", bReadEH);
        spinner = new CarouselSpinner(hardwareMap, "leftSpinner", "rightSpinner");

        tseDetector = new OpenCV(hardwareMap);//, dashboard);
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

        POS_ACC = 1;
        SLOW_DIST = 15;

        //Drive to the duckwheel
        driveTo(5, 4, 0);

        //Spin and stop duckwheel
        spinner.setLeftPower(0.75);
        waitForTime(4000);
        spinner.setLeftPower(0);

        //Drive to hub
        driveTo(34,-26, Math.toRadians(307));

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
        driveTo( 36,-31, Math.toRadians(300));
        outtake();

        H_ACC = Math.toRadians(3);

        //Drive a little bit back and drop lift
        driveTo(32,-26, Math.toRadians(300));
        lift.setTargetHeight(Lift.GROUND_HEIGHT);

        while (getRuntime() < 22) {
            updateInputs();
            updateOutputs();
        }
        lift.setTargetHeight(Lift.BARRIER_HEIGHT);

        //Align with the warehouse and park
        driveTo(30,-33, Math.toRadians(100));
        setLiftHeight(Lift.BARRIER_HEIGHT);

        SLOW_DIST = 20;
        driveTo(40,-128, Math.toRadians(100));

        //Lower lift all the way down for TeleOp
        setLiftHeight(Lift.GROUND_HEIGHT);

        /// Stop ///

        intake.stop();
        lift.stop();
        spinner.stop();
        setMotorPowers(0, 0, 0, 0);

        waitForTime(1000);
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