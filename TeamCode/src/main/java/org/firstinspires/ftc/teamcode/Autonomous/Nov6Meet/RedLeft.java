package org.firstinspires.ftc.teamcode.Autonomous.Nov6Meet;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Autonomous_Base;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.CarouselSpinner;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.TurningIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam.OpenCV;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam.TSEPipeline;
import org.opencv.core.RotatedRect;

@Autonomous(name = "Red Left")
//@Disabled
public class RedLeft extends Autonomous_Base {

    private TurningIntake turningIntake;
    private Lift lift;
    private CarouselSpinner spinner;

    private int liftLevel = 0;

    private OpenCV tseDetector;

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

        tseDetector = new OpenCV(hardwareMap);
        tseDetector.start(new TSEPipeline(0, 0, 640, 360));

        /// Start ///

        waitForStart();

        detectBarcode();
        tseDetector.stop();

        if (isStopRequested()) return;

        /// Loop ///

        //Vision

        //Drive to the duckwheel
        driveTo(4, 5, 0);

        //Spin and stop duckwheel
        spinner.setRightPower(0.75);
        waitForTime(4000);
        spinner.setRightPower(0);

        //Drive to hub
        driveTo(40,-28,315);

        //Put lift up
        setLiftHeight(liftLevel);

        //Drive to hub and outtake
        driveTo(42,-30,315);
        turningIntake.setIntakePower(-1);
        waitForTime(750);
        turningIntake.setIntakePower(0);

        //Drive a little bit back and drop lift
        driveTo(40,-28,315);
        setLiftHeight(2);

        //Align with the warehouse and park
        driveTo(33,-26,270);
        driveTo(33,-75,270);
    }

    private HubLevel detectBarcode() {
        RotatedRect boundingRect = tseDetector.getRect();
        if (boundingRect.center.x <= 640 / 3.0) {
            return HubLevel.LOW;
        }
        else if (boundingRect.center.x >= 2 * 640 / 3.0) {
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