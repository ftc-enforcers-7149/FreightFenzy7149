package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.CarouselSpinner;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam.OpenCV;
import org.opencv.core.RotatedRect;

public class AutoCommands {

    private final Autonomous_Base op;
    private final boolean USE_SUBS;

    public AutoCommands(Autonomous_Base op) {
        this.op = op;
        USE_SUBS = op.USE_SUBS;
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
        if (!USE_SUBS) return;
        lift.setTargetHeight(height);
        while (op.opModeIsActive() && lift.getLiftHeight() < height - 0.5) {
            op.updateInputs();
            op.updateOutputs();
        }
    }

    public void outtake(Intake intake, long msTime) {
        if (!USE_SUBS) intake.setIntakePower(-1);
        op.waitForTime(msTime);
        if (!USE_SUBS) intake.setIntakePower(0);
    }

    public void spinDuck(CarouselSpinner spinner, long msTime) {
        if (op.getAlliance() == Alliance.RED) {
            if (!USE_SUBS) spinner.setLeftPower(0.75);
            op.waitForTime(msTime);
            if (!USE_SUBS) spinner.setLeftPower(0);
        }
        else if (op.getAlliance() == Alliance.BLUE) {
            if (!USE_SUBS) spinner.setRightPower(0.75);
            op.waitForTime(msTime);
            if (!USE_SUBS) spinner.setRightPower(0);
        }
    }


}
