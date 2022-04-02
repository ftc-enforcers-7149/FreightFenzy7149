package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.CarouselSpinner;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.FourBar;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.MotorCarouselSpinner;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.MotorIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors.DistanceCorrection;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Input;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam.OpenCV;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam.TSEPipeline;

import static org.firstinspires.ftc.teamcode.GlobalData.ALLIANCE;
import static org.firstinspires.ftc.teamcode.GlobalData.HEADING;
import static org.firstinspires.ftc.teamcode.GlobalData.RAN_AUTO;

public abstract class Auto_V2_5 extends Autonomous_Base {

    protected MotorIntake intake;
    protected Lift lift;
    protected FourBar fourBar;
    protected MotorCarouselSpinner spinner;
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
        intake = new MotorIntake(hardwareMap,
                "intake", "paddle", "latch", "intakeColor");
        lift = new Lift(hardwareMap, "lift", bReadCH, !RAN_AUTO);
        fourBar = new FourBar(hardwareMap, "fourBarL", "fourBarR", "counterL", "counterR");
        spinner = new MotorCarouselSpinner(hardwareMap, "spinner", getAlliance());
        distCorrect = new DistanceCorrection(hardwareMap, "distL", "distR","distF", bReadEH, getAlliance());

        distCorrect.setQuartileSmoothing(true);

        //Add inputs & outputs
        addInput(intake);
        addInput(lift);
        addInput(spinner);
        addInput(distCorrect);
        addOutput(intake);
        addOutput(lift);
        addOutput(spinner);
        addOutput(fourBar);

        //Update global headless data as an input
        addInput(new Input() {
            @Override
            public void updateInput() {
                HEADING = drive.getPoseEstimate().getHeading();
            }
        });

        //Initialize intake servo positions
        intake.setLatch(MotorIntake.LatchPosition.CLOSED);
        intake.setPaddle(MotorIntake.PaddlePosition.BACK);
        intake.startOutput();
        intake.updateOutput();
        fourBar.startOutput();
        fourBar.updateOutput();

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
        fourBar.goToAngle(0);
        intake.setIntakePower(0);
        intake.setPaddle(MotorIntake.PaddlePosition.BACK);
        intake.setLatch(MotorIntake.LatchPosition.OPEN);

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
        telemetry.addData("Four Bar Angle: ", fourBar.getCurrAngle());
        telemetry.addData("Intake Distance: ", intake.getDistance());
        telemetry.addData("Freight in Intake? ", intake.getFreightInIntake());
    }
}
