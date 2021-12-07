package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.CarouselSpinner;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.ElevatorOld;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam.OpenCV;
import org.opencv.core.RotatedRect;

import static org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.ElevatorOld.Level.HIGH;
import static org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.ElevatorOld.Level.LOW;
import static org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.ElevatorOld.Level.MIDDLE;

public class AutoCommands {

    private final Autonomous_Base op;

    public AutoCommands(Autonomous_Base op) {
        this.op = op;
    }

    public ElevatorOld.Level detectBarcode(OpenCV tseDetector) {
        if (op.getAlliance() == Alliance.BLUE) {
            RotatedRect boundingRect = tseDetector.getRect();
            if (boundingRect == null) return LOW;
            if (boundingRect.center.x <= 640 / 4.0) {
                return MIDDLE;
            } else {
                return HIGH;
            }
        }
        else if (op.getAlliance() == Alliance.RED) {
            RotatedRect boundingRect = tseDetector.getRect();
            if (boundingRect == null) return HIGH;
            if (boundingRect.center.x <= 640 / 4.0) {
                return LOW;
            } else {
                return MIDDLE;
            }
        }
        else {
            return HIGH;
        }
    }

    public void setElevatorHeight(ElevatorOld elevator, double height) {
        elevator.setTargetHeight(height);
        op.customWait(() -> (elevator.getHeight() < height - 0.5));
    }

    public void setElevatorHeight(ElevatorOld elevator, ElevatorOld.Level level) {
        setElevatorHeight(elevator, level.height);
    }

    public void outtake(Intake intake) {
        intake.setIntakePower(-0.75);
        long startTime = System.currentTimeMillis();
        op.customWait(() -> (intake.getFreightInIntake() && System.currentTimeMillis() < startTime + 1500));
        intake.setIntakePower(0);
    }

    public void spinDuck(CarouselSpinner spinner, long msTime) {
        spinner.setPower(op.getAlliance(), 1);
        op.waitForTime(msTime);
        spinner.setPower(op.getAlliance(), 0);
    }
}
