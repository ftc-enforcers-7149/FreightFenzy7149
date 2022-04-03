package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.GlobalData;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.ArmController;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.FourBar;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.MotorCarouselSpinner;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.MotorIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors.DistanceCorrection;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Input;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.LED.LED;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Output;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam.OpenCV;

import static org.firstinspires.ftc.teamcode.GlobalData.ALLIANCE;
import static org.firstinspires.ftc.teamcode.GlobalData.HEADING;
import static org.firstinspires.ftc.teamcode.GlobalData.RAN_AUTO;
import static org.firstinspires.ftc.teamcode.GlobalData.openSignal;

public abstract class Auto_V2_5 extends Autonomous_Base {

    protected MotorIntake intake;
    protected Lift lift;
    protected FourBar fourBar;
    protected ArmController armController;
    protected MotorCarouselSpinner spinner;
    protected DistanceCorrection distCorrect;

    protected LED led;

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

        distCorrect.setQuartileSmoothing(false);

        armController = new ArmController(lift, fourBar);

        led = new LED(hardwareMap, "blinkin", Alliance.BLUE);

        if (getAlliance() == Alliance.BLUE)
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);
        else
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_RED);

        led.updateOutput();

        //Add inputs & outputs
        addInput(intake);
        addInput(lift);
        addInput(spinner);
        addInput(distCorrect);
        addOutput(intake);
        addOutput(lift);
        addOutput(spinner);
        addOutput(fourBar);
        addOutput(led);

        //Update global headless data as an input
        addInput(new Input() {
            @Override
            public void updateInput() {
                HEADING = drive.getPoseEstimate().getHeading();
            }
        });

        //Handle open signalling
        addOutput(new Output() {
            long startTime = 0;
            boolean running = false;

            @Override
            public void updateOutput() {
                if (GlobalData.openSignal) {
                    startTime = System.currentTimeMillis();
                    running = true;
                    GlobalData.openSignal = false;
                }

                if (running) {
                    if (System.currentTimeMillis() < startTime + 150)
                        intake.setLatch(MotorIntake.LatchPosition.OPEN);
                    else
                        running = false;
                }
            }

            @Override
            public void stopOutput() {
                GlobalData.openSignal = false;
            }
        });

        //Handle outtake signalling
        addOutput(new Output() {
            long startTime = 0;
            boolean running = false;

            @Override
            public void updateOutput() {
                if (GlobalData.outtakeSignal) {
                    startTime = System.currentTimeMillis();
                    running = true;
                    GlobalData.outtakeSignal = false;
                }

                if (running) {
                    if (System.currentTimeMillis() < startTime + 150)
                        intake.setPaddle(MotorIntake.PaddlePosition.OUT_FAR);
                    else {
                        intake.setPaddle(MotorIntake.PaddlePosition.BACK);
                        running = false;
                    }
                }
            }

            @Override
            public void stopOutput() {
                GlobalData.outtakeSignal = false;
            }
        });

        //Handle intake signalling
        addOutput(new Output() {
            @Override
            public void updateOutput() {
                if (GlobalData.intakeSignal)
                    intake.setIntakePower(1);
            }

            @Override
            public void stopOutput() {
                GlobalData.intakeSignal = false;
            }
        });

        //Handle arm up signalling
        addOutput(new Output() {
            @Override
            public void updateOutput() {
                if (GlobalData.armUpSignal) {
                    armController.setScorePos(ArmController.ScoringPosition.UP);
                    GlobalData.armUpSignal = false;
                }
            }
        });

        //Handle arm out signalling
        addOutput(new Output() {
            @Override
            public void updateOutput() {
                if (GlobalData.armOutSignal) {
                    armController.setScorePos(ArmController.ScoringPosition.HIGH);
                    GlobalData.armOutSignal = false;
                }
            }
        });

        //Handle arm in signalling
        addOutput(new Output() {
            long startTime = 0;
            boolean running = false;

            @Override
            public void updateOutput() {
                if (GlobalData.armInSignal) {
                    startTime = System.currentTimeMillis();
                    running = true;
                    GlobalData.armInSignal = false;
                }

                if (running) {
                    if (System.currentTimeMillis() < startTime + 150) {
                        armController.setScorePos(ArmController.ScoringPosition.IN);
                        if (intake.getIntakePower() == 0)
                            intake.setIntakePower(-1);
                    }
                    else {
                        if (intake.getIntakePower() == -1)
                            intake.setIntakePower(0);
                        running = false;
                    }
                }
            }
        });

        //Initialize intake servo positions
        intake.setLatch(MotorIntake.LatchPosition.CLOSED);
        intake.setPaddle(MotorIntake.PaddlePosition.BACK);
        intake.startOutput();
        intake.updateOutput();

        //Initialize vision for either alliance
        tseDetector = new OpenCV(hardwareMap);
        //tseDetector.start(new TSEPipeline(0, 350, 360, 100));

        /// Init Loop ///

        //Check vision
        while (!isStarted() && !isStopRequested()) {
            //tseDetector.update();
            //telemetry.addData("Hub Level: ", commands.detectBarcode(tseDetector));
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
        fourBar.setPosition(0);
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
