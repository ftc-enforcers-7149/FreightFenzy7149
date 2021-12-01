package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Odometry.DriveWheels.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.CarouselSpinner;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors.Positioning;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam.OpenCV;
import org.opencv.core.RotatedRect;

import static org.firstinspires.ftc.teamcode.Autonomous.Autonomous_Base.CLOSE_DIST;
import static org.firstinspires.ftc.teamcode.Autonomous.Autonomous_Base.H_ACC;
import static org.firstinspires.ftc.teamcode.Autonomous.Autonomous_Base.H_PID;
import static org.firstinspires.ftc.teamcode.Autonomous.Autonomous_Base.MIN_SPEED;
import static org.firstinspires.ftc.teamcode.Autonomous.Autonomous_Base.MIN_TURN;
import static org.firstinspires.ftc.teamcode.Autonomous.Autonomous_Base.POS_ACC;
import static org.firstinspires.ftc.teamcode.Autonomous.Autonomous_Base.SLOW_DIST;

public class AutoCommands {

    private final Autonomous_Base op;

    public AutoCommands(Autonomous_Base op) {
        this.op = op;
    }

    public HubLevel detectBarcode(OpenCV tseDetector) {
        RotatedRect boundingRect = tseDetector.getRect();
        if (boundingRect == null) return HubLevel.LOW;
        if (boundingRect.center.x <= 640 / 4.0) {
            return HubLevel.MIDDLE;
        }
        else {
            return HubLevel.HIGH;
        }
    }

    public void setLiftHeight(Lift lift, double height) {
        lift.setTargetHeight(height);
        op.customWait(() -> (lift.getLiftHeight() < height - 0.5));
    }

    public void outtake(Intake intake, long msTime) {
        intake.setIntakePower(-0.75);
        op.waitForTime(msTime);
        intake.setIntakePower(0);
    }

    public void spinDuck(CarouselSpinner spinner, long msTime) {
        if (op.getAlliance() == Alliance.RED) {
            spinner.setLeftPower(0.75);
            op.waitForTime(msTime);
            spinner.setLeftPower(0);
        }
        else if (op.getAlliance() == Alliance.BLUE) {
            spinner.setRightPower(0.75);
            op.waitForTime(msTime);
            spinner.setRightPower(0);
        }
    }

    //TODO: Assuming color sensor in FRONT of robot
    public void cycle(MecanumDrive drive, Positioning positioning,
                      Lift lift, Intake intake, boolean stopWithin) {
        driveToGap(lift, stopWithin);
        driveThroughGap(drive, positioning, stopWithin);
        driveToFreightAndBack(drive, positioning, intake, stopWithin);
        driveToHub(lift, intake);
    }

    public void driveToGap(Lift lift, boolean stop) {
        lift.setTargetHeight(Lift.GROUND_HEIGHT);

        if (op.getAlliance() == Alliance.RED) op.driveTo(5, -24, 270, stop);
        else if (op.getAlliance() == Alliance.BLUE) op.driveTo(5, 24, 90, stop);
    }
    public void driveThroughGap(MecanumDrive drive, Positioning positioning, boolean stop) {
        double destX = drive.getPoseEstimate().getX(),
                destY = drive.getPoseEstimate().getY(),
                destH = drive.getPoseEstimate().getHeading();

        if (op.getAlliance() == Alliance.RED) {
            destX = drive.getPoseEstimate().getX() - positioning.getRightDistance();
            destY = drive.getPoseEstimate().getY() - 20;
            destH = Math.toRadians(270);
        }
        else if (op.getAlliance() == Alliance.BLUE) {
            destX = drive.getPoseEstimate().getX() - positioning.getLeftDistance();
            destY = drive.getPoseEstimate().getY() + 20;
            destH = Math.toRadians(90);
        }

        //Initial state
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PIDFController hControl = new PIDFController(H_PID);
        hControl.setOutputBounds(0, 1);
        hControl.setTargetPosition(0);

        //Current robot position
        double robotX = drive.getPoseEstimate().getX();
        double robotY = drive.getPoseEstimate().getY();
        double robotH = drive.getPoseEstimate().getHeading();

        //Calculate relatives
        double relX = destX - robotX;
        double relY = destY - robotY;
        double relH = op.deltaHeading(robotH, destH);

        double hWeight;

        //Drive to line
        while (op.opModeIsActive() &&
                (Math.abs(relX) > 3 ||
                        Math.abs(relY) > POS_ACC ||
                        Math.abs(relH) > Math.toRadians(5))) {

            op.updateInputs();

            //Check wall and color sensor, adjust destination accordingly
            if (op.getAlliance() == Alliance.RED) {
                destX = drive.getPoseEstimate().getX() - positioning.getRightDistance();
                if (positioning.getLineDetected()) destY = drive.getPoseEstimate().getY();
                else destY = drive.getPoseEstimate().getY() - 20;
            }
            else if (op.getAlliance() == Alliance.BLUE) {
                destX = drive.getPoseEstimate().getX() - positioning.getLeftDistance();
                if (positioning.getLineDetected()) destY = drive.getPoseEstimate().getY();
                else destY = drive.getPoseEstimate().getY() + 20;
            }

            //Normal driveTo code
            //Update robot position
            robotX = drive.getPoseEstimate().getX();
            robotY = drive.getPoseEstimate().getY();
            robotH = drive.getPoseEstimate().getHeading();

            //Calculate relatives
            relX = destX - robotX;
            relY = destY - robotY;
            relH = op.deltaHeading(robotH, destH);

            hWeight = hControl.update(Math.abs(relH));

            if (Math.abs(relX) < POS_ACC) relX = 0;
            if (Math.abs(relY) < POS_ACC) relY = 0;
            if (Math.abs(relH) < H_ACC) hWeight = 0;

            double driveAngle = op.deltaHeading(robotH, Math.atan2(relY, relX));

            double xPower = Math.cos(driveAngle);
            double yPower = Math.sin(driveAngle);
            double hPower = Math.copySign(hWeight, relH);

            double dist = Math.sqrt((relX*relX) + (relY*relY));

            if (dist <= (stop ? SLOW_DIST : -1)) {
                xPower *= Math.pow(dist / SLOW_DIST, 2);
                yPower *= Math.pow(dist / SLOW_DIST, 2);
            }

            double max = Math.max(Math.abs(xPower), Math.abs(yPower));
            if (dist <= (stop ? CLOSE_DIST : -1) && max != 0) {
                xPower /= max / MIN_SPEED;
                yPower /= max / MIN_SPEED;
            }
            else {
                if (max < MIN_SPEED && max != 0) {
                    xPower /= max / MIN_SPEED;
                    yPower /= max / MIN_SPEED;
                }
                if (Math.abs(hPower) < MIN_TURN && Math.abs(hPower) > 0)
                    hPower = Math.copySign(MIN_TURN, hPower);
            }

            drive.setWeightedDrivePower(new Pose2d(xPower, yPower, hPower));

            op.telemetry.addData("X Power: ", xPower);
            op.telemetry.addData("Y Power: ", yPower);
            op.telemetry.addData("H Power: ", hPower);

            op.updateOutputs();
        }

        //TODO: Get right positions
        if (op.getAlliance() == Alliance.RED) drive.setPoseEstimate(new Pose2d(0, -48, 270));
        else if (op.getAlliance() == Alliance.BLUE) drive.setPoseEstimate(new Pose2d(0, 48, 90));

        if (stop) op.setMotorPowers(0, 0, 0, 0);
    }
    public void driveToFreightAndBack(MecanumDrive drive, Positioning positioning,
                                      Intake intake, boolean stop) {
        intake.setIntakePower(1);
        positioning.startLineDetector();

        double destX = drive.getPoseEstimate().getX(),
                destY = drive.getPoseEstimate().getY(),
                destH = drive.getPoseEstimate().getHeading();

        if (op.getAlliance() == Alliance.RED) {
            destX = drive.getPoseEstimate().getX() - positioning.getRightDistance();
            destY = drive.getPoseEstimate().getY() - 20;
            destH = Math.toRadians(270);
        }
        else if (op.getAlliance() == Alliance.BLUE) {
            destX = drive.getPoseEstimate().getX() - positioning.getLeftDistance();
            destY = drive.getPoseEstimate().getY() + 20;
            destH = Math.toRadians(90);
        }

        //Initial state
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PIDFController hControl = new PIDFController(H_PID);
        hControl.setOutputBounds(0, 1);
        hControl.setTargetPosition(0);

        //Current robot position
        double robotX = drive.getPoseEstimate().getX();
        double robotY = drive.getPoseEstimate().getY();
        double robotH = drive.getPoseEstimate().getHeading();

        //Calculate relatives
        double relX = destX - robotX;
        double relY = destY - robotY;
        double relH = op.deltaHeading(robotH, destH);

        double hWeight;

        //Drive to line
        TO: while (op.opModeIsActive() &&
                (Math.abs(relX) > 3 ||
                        Math.abs(relY) > POS_ACC ||
                        Math.abs(relH) > Math.toRadians(5))) {

            op.updateInputs();

            //Check wall and color sensor, adjust destination accordingly
            if (op.getAlliance() == Alliance.RED) {
                destX = drive.getPoseEstimate().getX() - positioning.getRightDistance();
                if (intake.getFreightInIntake()) break TO;
                else destY = drive.getPoseEstimate().getY() - 20;
            }
            else if (op.getAlliance() == Alliance.BLUE) {
                destX = drive.getPoseEstimate().getX() - positioning.getLeftDistance();
                if (intake.getFreightInIntake()) break TO;
                else destY = drive.getPoseEstimate().getY() + 20;
            }

            //Normal driveTo code
            //Update robot position
            robotX = drive.getPoseEstimate().getX();
            robotY = drive.getPoseEstimate().getY();
            robotH = drive.getPoseEstimate().getHeading();

            //Calculate relatives
            relX = destX - robotX;
            relY = destY - robotY;
            relH = op.deltaHeading(robotH, destH);

            hWeight = hControl.update(Math.abs(relH));

            if (Math.abs(relX) < POS_ACC) relX = 0;
            if (Math.abs(relY) < POS_ACC) relY = 0;
            if (Math.abs(relH) < H_ACC) hWeight = 0;

            double driveAngle = op.deltaHeading(robotH, Math.atan2(relY, relX));

            double xPower = Math.cos(driveAngle);
            double yPower = Math.sin(driveAngle);
            double hPower = Math.copySign(hWeight, relH);

            double dist = Math.sqrt((relX*relX) + (relY*relY));

            if (dist <= SLOW_DIST) {
                xPower *= Math.pow(dist / SLOW_DIST, 2);
                yPower *= Math.pow(dist / SLOW_DIST, 2);
            }

            double max = Math.max(Math.abs(xPower), Math.abs(yPower));
            if (dist <= CLOSE_DIST && max != 0) {
                xPower /= max / MIN_SPEED;
                yPower /= max / MIN_SPEED;
            }
            else {
                if (max < MIN_SPEED && max != 0) {
                    xPower /= max / MIN_SPEED;
                    yPower /= max / MIN_SPEED;
                }
                if (Math.abs(hPower) < MIN_TURN && Math.abs(hPower) > 0)
                    hPower = Math.copySign(MIN_TURN, hPower);
            }

            drive.setWeightedDrivePower(new Pose2d(xPower, yPower, hPower));

            op.telemetry.addData("X Power: ", xPower);
            op.telemetry.addData("Y Power: ", yPower);
            op.telemetry.addData("H Power: ", hPower);

            op.updateOutputs();
        }

        //intake.setIntakePower(0);

        destX = drive.getPoseEstimate().getX();
        destY = drive.getPoseEstimate().getY();
        destH = drive.getPoseEstimate().getHeading();

        if (op.getAlliance() == Alliance.RED) {
            destX = drive.getPoseEstimate().getX() - positioning.getRightDistance();
            destY = drive.getPoseEstimate().getY() + 20;
            destH = Math.toRadians(270);
        }
        else if (op.getAlliance() == Alliance.BLUE) {
            destX = drive.getPoseEstimate().getX() - positioning.getLeftDistance();
            destY = drive.getPoseEstimate().getY() - 20;
            destH = Math.toRadians(90);
        }

        //Initial state
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hControl = new PIDFController(H_PID);
        hControl.setOutputBounds(0, 1);
        hControl.setTargetPosition(0);

        //Current robot position
        robotX = drive.getPoseEstimate().getX();
        robotY = drive.getPoseEstimate().getY();
        robotH = drive.getPoseEstimate().getHeading();

        //Calculate relatives
        relX = destX - robotX;
        relY = destY - robotY;
        relH = op.deltaHeading(robotH, destH);

        BACK: while (op.opModeIsActive() &&
                (Math.abs(relX) > 3 ||
                        Math.abs(relY) > POS_ACC ||
                        Math.abs(relH) > Math.toRadians(5))) {

            op.updateInputs();

            //Check wall and color sensor, adjust destination accordingly
            if (op.getAlliance() == Alliance.RED) {
                destX = drive.getPoseEstimate().getX() - positioning.getRightDistance();
                if (positioning.getLineDetected()) break BACK;
                else destY = drive.getPoseEstimate().getY() + 20;
            }
            else if (op.getAlliance() == Alliance.BLUE) {
                destX = drive.getPoseEstimate().getX() - positioning.getLeftDistance();
                if (positioning.getLineDetected()) break BACK;
                else destY = drive.getPoseEstimate().getY() - 20;
            }

            //Normal driveTo code
            //Update robot position
            robotX = drive.getPoseEstimate().getX();
            robotY = drive.getPoseEstimate().getY();
            robotH = drive.getPoseEstimate().getHeading();

            //Calculate relatives
            relX = destX - robotX;
            relY = destY - robotY;
            relH = op.deltaHeading(robotH, destH);

            hWeight = hControl.update(Math.abs(relH));

            if (Math.abs(relX) < POS_ACC) relX = 0;
            if (Math.abs(relY) < POS_ACC) relY = 0;
            if (Math.abs(relH) < H_ACC) hWeight = 0;

            double driveAngle = op.deltaHeading(robotH, Math.atan2(relY, relX));

            double xPower = Math.cos(driveAngle);
            double yPower = Math.sin(driveAngle);
            double hPower = Math.copySign(hWeight, relH);

            double dist = Math.sqrt((relX*relX) + (relY*relY));

            if (dist <= SLOW_DIST) {
                xPower *= Math.pow(dist / SLOW_DIST, 2);
                yPower *= Math.pow(dist / SLOW_DIST, 2);
            }

            double max = Math.max(Math.abs(xPower), Math.abs(yPower));
            if (dist <= CLOSE_DIST && max != 0) {
                xPower /= max / MIN_SPEED;
                yPower /= max / MIN_SPEED;
            }
            else {
                if (max < MIN_SPEED && max != 0) {
                    xPower /= max / MIN_SPEED;
                    yPower /= max / MIN_SPEED;
                }
                if (Math.abs(hPower) < MIN_TURN && Math.abs(hPower) > 0)
                    hPower = Math.copySign(MIN_TURN, hPower);
            }

            drive.setWeightedDrivePower(new Pose2d(xPower, yPower, hPower));

            op.telemetry.addData("X Power: ", xPower);
            op.telemetry.addData("Y Power: ", yPower);
            op.telemetry.addData("H Power: ", hPower);

            op.updateOutputs();
        }

        //TODO: Get right positions
        if (op.getAlliance() == Alliance.RED) drive.setPoseEstimate(new Pose2d(0, -48, 270));
        else if (op.getAlliance() == Alliance.BLUE) drive.setPoseEstimate(new Pose2d(0, 48, 90));

        positioning.stopLineDetector();

        //Finish driving through gap

        if (op.getAlliance() == Alliance.RED)
            op.driveTo(drive.getPoseEstimate().getX(), -24, 270, stop);
        else if (op.getAlliance() == Alliance.BLUE)
            op.driveTo(drive.getPoseEstimate().getX(), 24, 90, stop);
    }
    public void driveToHub(Lift lift, Intake intake) {
        lift.setTargetHeight(Lift.HIGH_HEIGHT);

        if (op.getAlliance() == Alliance.RED) op.driveTo(35, -8, 45);
        else if (op.getAlliance() == Alliance.BLUE) op.driveTo(35, 8, 315);

        setLiftHeight(lift, Lift.HIGH_HEIGHT);

        outtake(intake, 500);
    }
}
