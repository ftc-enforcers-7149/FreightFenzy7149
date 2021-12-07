package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.CarouselSpinner;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Scorer;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Input;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam.OpenCV;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam.TSEPipeline;

import static org.firstinspires.ftc.teamcode.GlobalData.HEADING;
import static org.firstinspires.ftc.teamcode.GlobalData.ALLIANCE;
import static org.firstinspires.ftc.teamcode.GlobalData.RAN_AUTO;

public abstract class Auto_V3 extends Autonomous_Base {

    protected CarouselSpinner spinner;
    protected Intake intake;
    protected Elevator elevator;
    protected Turret turret;
    protected Scorer scorer;

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

        spinner = new CarouselSpinner(hardwareMap, "spinner");
        intake = new Intake(hardwareMap, "intake", "intakeColor");
        elevator = new Elevator(hardwareMap, "elevator", bReadEH);
        turret = new Turret(hardwareMap, "turret", bReadEH);
        scorer = new Scorer(elevator, turret);

        addInput(intake);
        addInput(elevator);
        addInput(turret);

        addOutput(spinner);
        addOutput(intake);
        addOutput(scorer);

        //Update global headless data as an input
        addInput(new Input() {
            @Override
            public void updateInput() {
                HEADING = gyro.getRawYaw();
            }
        });

        tseDetector = new OpenCV(hardwareMap);
        if (getAlliance() == Alliance.RED)
            tseDetector.start(new TSEPipeline(320, 180, 320, 180));
        else
            tseDetector.start(new TSEPipeline(0, 180, 320, 180));

        startInputs();
        startOutputs();

        /// Init Loop ///

        while (!isStarted() && !isStopRequested()) {
            tseDetector.update();
            telemetry.addData("Hub Level: ", commands.detectBarcode(tseDetector));
            telemetry.update();
        }
        if (isStopRequested()) return;

        /// Start ///

        resetStartTime();

        //Set global variables
        ALLIANCE = getAlliance();
        RAN_AUTO = true;
        HEADING = gyro.getRawYaw();

        tseDetector.stop();

        startInputs();
        startOutputs();

        /// Loop ///

        auto();

        /// Stop ///

        stopInputs();
        stopOutputs();

        waitForTime(500);
    }

    @Override
    protected final void addTelemetryData() {
        telemetry.addData("Position: ", drive.getPoseEstimate());
        telemetry.addData("Freight in Intake? ", intake.getFreightInIntake());
    }
}
