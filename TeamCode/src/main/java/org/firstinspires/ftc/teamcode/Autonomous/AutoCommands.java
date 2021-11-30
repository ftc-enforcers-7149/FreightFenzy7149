package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.CarouselSpinner;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.ElevatorOld;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam.OpenCV;
import org.opencv.core.RotatedRect;

public class AutoCommands {

    private final Autonomous_Base op;

    public AutoCommands(Autonomous_Base op) {
        this.op = op;
    }

    public ElevatorOld.Level detectBarcode(OpenCV tseDetector) {
        RotatedRect boundingRect = tseDetector.getRect();
        if (boundingRect == null) return ElevatorOld.Level.LOW;
        if (boundingRect.center.x <= 640 / 4.0) {
            return ElevatorOld.Level.MIDDLE;
        }
        else {
            return ElevatorOld.Level.HIGH;
        }
    }

    public void setElevatorHeight(ElevatorOld elevator, double height) {
        elevator.setTargetHeight(height);
        op.customWait(() -> (elevator.getHeight() < height - 0.5));
    }

    public void setElevatorHeight(ElevatorOld elevator, ElevatorOld.Level level) {
        setElevatorHeight(elevator, level.height);
    }

    public void outtake(Intake intake, long msTime) {
        intake.setIntakePower(-1);
        op.waitForTime(msTime);
        intake.setIntakePower(0);
    }

    public void spinDuck(CarouselSpinner spinner, long msTime) {
        spinner.setPower(op.getAlliance(), 1);
        op.waitForTime(msTime);
        spinner.setPower(op.getAlliance(), 0);
    }
}
