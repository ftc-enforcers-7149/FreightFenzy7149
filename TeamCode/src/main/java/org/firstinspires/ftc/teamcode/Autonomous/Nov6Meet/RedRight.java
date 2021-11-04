package org.firstinspires.ftc.teamcode.Autonomous.Nov6Meet;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Autonomous_Base;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.CarouselSpinner;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.TurningIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam.OpenCV;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam.TSEPipeline;
import org.opencv.core.RotatedRect;

@Autonomous(name = "Red Right")
//@Disabled
public class RedRight extends Autonomous_Base {

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

        //Add Vision

        //Set lift to correct level according to the vision
        setLiftHeight(liftLevel);

        //Drive to hub
        driveTo(16,0,0);

        //Deliver pre-loaded block
        turningIntake.setIntakePower(-1);
        waitForTime(750);
        turningIntake.setIntakePower(0);

        //Move back a little so that the intake dosen't hit the hub
        driveTo(14,0,0);

        //Put lift back down
        setLiftHeight(0);

        //Realign with the wall and turn towards the warehouse
        driveTo(0,0,270);

        //Start intakeing
        turningIntake.setIntakePower(1);

        //Drive into the warehouse
        driveTo(0,-47,270);

        //Stop intake
        turningIntake.setIntakePower(0);

        //Drive backwards to the hub
        driveTo(0,0,270);

        //Turn and move towards the hub
        driveTo(14,0,0);

        //Set lift to the highest height
        setLiftHeight(16.5);
        //lift.setTargetHeight(3);

        //Move forward
        driveTo(16,0,0);

        //Outtake the game element
        turningIntake.setIntakePower(-1);
        waitForTime(750);
        turningIntake.setIntakePower(0);

        //Drive a little back and turn
        driveTo(14,0,270);

        //Put lift down
        lift.setTargetHeight(0);

        //Park in warehouse
        driveTo(0,0,270);
        driveTo(0,47,270);
    }

    private void outtake() {
        turningIntake.setIntakePower(-1);
        waitForTime(1000);
        turningIntake.setIntakePower(0);
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