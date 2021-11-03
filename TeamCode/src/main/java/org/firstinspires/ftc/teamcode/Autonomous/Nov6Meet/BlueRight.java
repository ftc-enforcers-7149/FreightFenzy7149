package org.firstinspires.ftc.teamcode.Autonomous.Nov6Meet;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Autonomous_Base;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.CarouselSpinner;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.TurningIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam.OpenCV;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam.TSEPipeline;
import org.opencv.core.RotatedRect;

@Autonomous(name = "Blue Right")
//@Disabled
public class BlueRight extends Autonomous_Base {

    private TurningIntake turningIntake;
    private Lift lift;
    private CarouselSpinner spinner;

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

        waitForStart();
        if (isStopRequested()) return;

        /// Loop ///

        //Drive into duckwheel
        POS_ACC = 1;
        driveTo(4, -5, 0);

        //Spin duckwheel while outtaking preloaded block into storage unit
        turningIntake.setWristRight();
        spinner.setRightPower(0.75);
        turningIntake.setIntakePower(-1);

        waitForTime(4000);

        //Stop spinning / outtaking
        spinner.setRightPower(0);
        turningIntake.setWristLeft();
        turningIntake.setIntakePower(0);

        waitForTime(750);

        //Park
        POS_ACC = 0.1;
        driveTo(29.5, -5, 0);

        /// Stop ///

        setMotorPowers(0, 0, 0, 0);

        turningIntake.stop();
        lift.stop();
        spinner.stop();

        waitForTime(1000);
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