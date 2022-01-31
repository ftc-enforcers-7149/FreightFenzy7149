package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.CarouselSpinner;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors.DistanceCorrection;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Input;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam.OpenCV;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam.TSEPipeline;

import static org.firstinspires.ftc.teamcode.GlobalData.ALLIANCE;
import static org.firstinspires.ftc.teamcode.GlobalData.HEADING;
import static org.firstinspires.ftc.teamcode.GlobalData.RAN_AUTO;

public abstract class Auto_V2_5 extends Autonomous_Base {

    protected Intake intake;
    protected Lift lift;
    protected CarouselSpinner spinner;
    protected DistanceCorrection distCorrect;

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

        //Initialize subsystems
        intake = new Intake(hardwareMap, "intake", "intakeColor");
        lift = new Lift(hardwareMap, "lift", bReadEH);
        spinner = new CarouselSpinner(hardwareMap, "spinner");
        distCorrect = new DistanceCorrection(hardwareMap, "distL", "distR", "distF", getAlliance());

        //Add inputs & outputs
        addInput(intake);
        addInput(lift);
        addInput(distCorrect);
        addOutput(intake);
        addOutput(lift);
        addOutput(spinner);

        //Update global headless data as an input
        addInput(new Input() {
            @Override
            public void updateInput() {
                HEADING = drive.getPoseEstimate().getHeading();
            }
        });

        //Initialize vision for either alliance
        tseDetector = new OpenCV(hardwareMap);
        tseDetector.start(new TSEPipeline(0, 350, 360, 100));

        /// Init Loop ///

        //Check vision
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
        HEADING = drive.getPoseEstimate().getHeading();

        tseDetector.stop();

        startInputs();
        startOutputs();

        /// Loop ///

        auto();

        setMotorPowers(0, 0, 0, 0);
        lift.setPower(0.05);
        intake.setIntakePower(0);

        customWait(() -> (!isStopRequested()));

        /// Stop ///

        stopInputs();
        stopOutputs();
    }

    @Override
    protected final void addTelemetryData() {
        telemetry.addData("Position: ", drive.getPoseEstimate());
        telemetry.addData("Front Distance: ", distCorrect.getFrontDistance());
        telemetry.addData("Side Distance: ", distCorrect.getSideWall());
        telemetry.addData("Lift Height: ", lift.getHeight());
        telemetry.addData("Freight in Intake? ", intake.getFreightInIntake());
    }
}
