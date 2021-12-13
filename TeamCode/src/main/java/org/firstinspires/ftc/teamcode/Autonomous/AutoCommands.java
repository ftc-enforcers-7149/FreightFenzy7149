package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.CarouselSpinner;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam.OpenCV;
import org.opencv.core.RotatedRect;

public class AutoCommands {

    private final Autonomous_Base op;

    public AutoCommands(Autonomous_Base op) {
        this.op = op;
    }

    public Elevator.Level detectBarcode(OpenCV tseDetector) {
        if (op.getAlliance() == Alliance.BLUE) {
            RotatedRect boundingRect = tseDetector.getRect();
            if (boundingRect == null) return Elevator.Level.LOW;
            if (boundingRect.center.x <= 640 / 4.0) {
                return Elevator.Level.MIDDLE;
            } else {
                return Elevator.Level.HIGH;
            }
        }
        else if (op.getAlliance() == Alliance.RED) {
            RotatedRect boundingRect = tseDetector.getRect();
            if (boundingRect == null) return Elevator.Level.HIGH;
            if (boundingRect.center.x <= 640 / 4.0) {
                return Elevator.Level.LOW;
            } else {
                return Elevator.Level.MIDDLE;
            }
        }
        else {
            return Elevator.Level.HIGH;
        }
    }

    public void setElevatorHeight(Elevator elevator, double height) {
        elevator.setTargetHeight(height);
        op.customWait(() -> (elevator.getHeight() < height - 0.5));
    }

    public void setElevatorHeight(Elevator elevator, Elevator.Level level) {
        setElevatorHeight(elevator, level.height);
    }

    public void outtake(Intake intake) {
        intake.outtakeUp();
        op.customWait(intake::isBusy);
    }

    public void spinDuck(CarouselSpinner spinner, long msTime) {
        spinner.setPower(op.getAlliance(), 1);
        op.waitForTime(msTime);
        spinner.setPower(op.getAlliance(), 0);
    }
}
