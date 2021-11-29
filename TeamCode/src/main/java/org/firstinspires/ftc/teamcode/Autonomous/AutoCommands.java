package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.CarouselSpinner;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam.OpenCV;
import org.opencv.core.RotatedRect;

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
        intake.setIntakePower(-1);
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
}
