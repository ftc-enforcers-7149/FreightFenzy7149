package org.firstinspires.ftc.teamcode.Autonomous;

import android.icu.text.MessagePattern;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Odometry.DriveWheels.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.CarouselSpinner;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors.Positioning;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam.OpenCV;
import org.opencv.core.RotatedRect;

import java.util.function.Supplier;

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
        if (op.getAlliance() == Alliance.BLUE) {
            RotatedRect boundingRect = tseDetector.getRect();
            if (boundingRect == null) return HubLevel.LOW;
            if (boundingRect.center.x <= 640 / 4.0) {
                return HubLevel.MIDDLE;
            } else {
                return HubLevel.HIGH;
            }
        }
        else if (op.getAlliance() == Alliance.RED) {
            RotatedRect boundingRect = tseDetector.getRect();
            if (boundingRect == null) return HubLevel.HIGH;
            if (boundingRect.center.x <= 640 / 4.0) {
                return HubLevel.LOW;
            } else {
                return HubLevel.MIDDLE;
            }
        }
        else {
            return HubLevel.HIGH;
        }
    }

    public void setLiftHeight(Lift lift, double height) {
        lift.setTargetHeight(height);
        op.customWait(() -> (lift.getLiftHeight() < height - 0.5));
    }

    public void outtake(Intake intake) {
        intake.setIntakePower(1);
        long startTime = System.currentTimeMillis();
        op.customWait(() -> (System.currentTimeMillis() < startTime + 750));
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

    public void cycle(MecanumDrive drive, Positioning positioning,
                      Lift lift, Intake intake) throws NoFreight {
        driveToGap(drive, positioning, lift);
        driveThroughGap(drive, positioning);
        driveToFreightAndBack(drive, positioning, intake);
        driveToHub(lift, intake);
    }

    public void driveToGap(MecanumDrive drive, Positioning positioning, Lift lift) {
        lift.setTargetHeight(Lift.GROUND_HEIGHT);
        driveWithWall(drive, positioning, 0, drive.getPoseEstimate()::getY);
    }
    public void driveThroughGap(MecanumDrive drive, Positioning positioning) {
        if (op.getAlliance() == Alliance.RED) {
            driveWithWall(drive, positioning, 0, () -> {
                if (positioning.getLineDetected()) return drive.getPoseEstimate().getY();
                else return -36.0;
            });
            if (positioning.getLineDetected())
                op.drive.setPoseEstimate(new Pose2d(0, -32, Math.toRadians(270)));
        }
        else if (op.getAlliance() == Alliance.BLUE) {
            driveWithWall(drive, positioning, 0, () -> {
                if (positioning.getLineDetected()) return drive.getPoseEstimate().getY();
                else return 36.0;
            });
            if (positioning.getLineDetected())
                op.drive.setPoseEstimate(new Pose2d(0, 32, Math.toRadians(90)));
        }
    }
    public void driveToFreightAndBack(MecanumDrive drive, Positioning positioning, Intake intake) throws NoFreight {
        intake.setIntakePower(-1);

        //TO
        if (op.getAlliance() == Alliance.RED) {
            driveWithWall(drive, positioning, 0, () -> {
                if (intake.getFreightInIntake()) return drive.getPoseEstimate().getY();
                else return -60.0;
            });
        }
        else if (op.getAlliance() == Alliance.BLUE) {
            driveWithWall(drive, positioning, 0, () -> {
                if (intake.getFreightInIntake()) return drive.getPoseEstimate().getY();
                else return 60.0;
            });
        }

        if (!intake.getFreightInIntake()) {
            throw new NoFreight();
        }

        intake.setIntakePower(-0.25);

        //BACK
        if (op.getAlliance() == Alliance.RED) {
            driveWithWall(drive, positioning, 0, () -> {
                if (positioning.getLineDetected()) return drive.getPoseEstimate().getY();
                else return -28.0;
            });
            if (positioning.getLineDetected())
                op.drive.setPoseEstimate(new Pose2d(0, -32, Math.toRadians(270)));
        }
        else if (op.getAlliance() == Alliance.BLUE) {
            driveWithWall(drive, positioning, 0, () -> {
                if (positioning.getLineDetected()) return drive.getPoseEstimate().getY();
                else return 28.0;
            });
            if (positioning.getLineDetected())
                op.drive.setPoseEstimate(new Pose2d(0, 32, Math.toRadians(90)));
        }

        //Finish driving through gap

        if (op.getAlliance() == Alliance.RED) {
            driveWithWall(drive, positioning, 0, () -> -20.0);
        }
        else if (op.getAlliance() == Alliance.BLUE) {
            driveWithWall(drive, positioning, 0, () -> 20.0);
        }
    }
    public void driveToHub(Lift lift, Intake intake) {
        lift.setTargetHeight(Lift.HIGH_HEIGHT);

        if (op.getAlliance() == Alliance.RED) op.driveTo(35, -8, Math.toRadians(45));
        else if (op.getAlliance() == Alliance.BLUE) op.driveTo(35, 8, Math.toRadians(315));

        setLiftHeight(lift, Lift.HIGH_HEIGHT);
        outtake(intake);
    }

    public void driveWithWall(MecanumDrive drive, Positioning positioning, double xOff, Supplier<Double> y) {
        positioning.startPositioning();
        if (xOff == 0) {
            H_ACC = Math.toRadians(3);
            POS_ACC = 2;
        }

        if (op.getAlliance() == Alliance.RED) {
            op.driveTo(
                    () -> {
                        if (positioning.getRightDistance() > 10) return 5.0;
                        else return drive.getPoseEstimate().getX() - positioning.getRightDistance() + xOff;
                    },
                    y,
                    () -> Math.toRadians(270));
            if (xOff == 0)
                op.drive.setPoseEstimate(new Pose2d(0, drive.getPoseEstimate().getY(), Math.toRadians(270)));
        }
        else if (op.getAlliance() == Alliance.BLUE) {
            op.driveTo(
                    () -> {
                        if (positioning.getRightDistance() > 10) return 5.0;
                        else return drive.getPoseEstimate().getX() - positioning.getRightDistance() + xOff;
                    },
                    y,
                    () -> Math.toRadians(90));
            if (xOff == 0)
                op.drive.setPoseEstimate(new Pose2d(0, drive.getPoseEstimate().getY(), Math.toRadians(90)));
        }

        if (xOff == 0) {
            H_ACC = Math.toRadians(1);
            POS_ACC = 1;
        }
        positioning.stopPositioning();
    }

    public class NoFreight extends Exception { }
}
