package org.firstinspires.ftc.teamcode.Autonomous;

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
        try {
            if (op.getAlliance() == Alliance.BLUE) {
                RotatedRect boundingRect = tseDetector.getRect();

                if (boundingRect == null || boundingRect.size.area() < 1500) return HubLevel.LOW;
                if (boundingRect.center.x <= 360 / 2.0) {
                    return HubLevel.MIDDLE;
                } else {
                    return HubLevel.HIGH;
                }
            } else if (op.getAlliance() == Alliance.RED) {
                RotatedRect boundingRect = tseDetector.getRect();
                if (boundingRect == null || boundingRect.size.area() < 1500) return HubLevel.HIGH;
                if (boundingRect.center.x <= 360 / 2.0) {
                    return HubLevel.LOW;
                } else {
                    return HubLevel.MIDDLE;
                }
            } else {
                return HubLevel.HIGH;
            }
        }
        catch(Exception e){
            if (op.getAlliance() == Alliance.BLUE)
                return HubLevel.LOW;
            else
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
        op.customWait(() -> (System.currentTimeMillis() < startTime + 1100));
        intake.setIntakePower(0);
    }

    public void outtake(Intake intake, long time) {
        intake.setIntakePower(1);
        long startTime = System.currentTimeMillis();
        op.customWait(() -> (System.currentTimeMillis() < startTime + time));
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
