package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.CarouselSpinner;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam.OpenCV;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam.TSEPipeline;

import static org.firstinspires.ftc.teamcode.GlobalData.ALLIANCE;
import static org.firstinspires.ftc.teamcode.GlobalData.HEADING;
import static org.firstinspires.ftc.teamcode.GlobalData.RAN_AUTO;

public abstract class Auto_V2 extends Autonomous_Base {

    protected Intake intake;
    protected Lift lift;
    protected CarouselSpinner spinner;

    protected OpenCV tseDetector;

    protected abstract void auto();

    @Override
    public final void runOpMode() throws InterruptedException {
        /// Init ///
        try {
            initializeAll();
        } catch (Exception e) {
            e.printStackTrace();
            throw new InterruptedException(e.getMessage());
        }

        intake = new Intake(hardwareMap, "intake", "intakeColor");
        lift = new Lift(hardwareMap, "lift", bReadEH);
        spinner = new CarouselSpinner(hardwareMap, "leftSpinner", "rightSpinner");

        addInput(intake);
        addInput(lift);
        addOutput(intake);
        addOutput(lift);
        addOutput(spinner);

        tseDetector = new OpenCV(hardwareMap);
        if (getAlliance() == Alliance.RED)
            tseDetector.start(new TSEPipeline(320, 180, 320, 180));
        else
            tseDetector.start(new TSEPipeline(0, 180, 320, 180));

        /// Init Loop ///

        while (!isStarted() && !isStopRequested()) {
            tseDetector.update();
            telemetry.addData("Hub Level: ", commands.detectBarcode(tseDetector));
            telemetry.update();
        }
        if (isStopRequested()) return;

        /// Start ///

        tseDetector.stop();

        startInputs();
        startOutputs();

        /// Loop ///

        auto();

        /// Stop ///

        stopInputs();
        stopOutputs();

        ALLIANCE = getAlliance();
        RAN_AUTO = true;
        HEADING = gyro.getRawYaw();

        waitForTime(500);
    }

    @Override
    protected final void addTelemetryData() {
        telemetry.addData("Position: ", drive.getPoseEstimate());
        telemetry.addData("Lift Height: ", lift.getLiftHeight());
    }
}
