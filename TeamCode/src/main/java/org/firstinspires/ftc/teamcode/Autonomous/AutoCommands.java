package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.CarouselSpinner;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Levels;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam.OpenCV;
import org.opencv.core.RotatedRect;

public class AutoCommands {

    private final Autonomous_Base op;

    public AutoCommands(Autonomous_Base op) {
        this.op = op;
    }

    public Levels detectBarcode(OpenCV tseDetector) {
        try {
            if (op.getAlliance() == Alliance.BLUE) {
                RotatedRect boundingRect = tseDetector.getRect();

                if (boundingRect == null || boundingRect.size.area() < 1500) return Levels.LOW;
                if (boundingRect.center.x <= 360 / 2.0) {
                    return Levels.MIDDLE;
                } else {
                    return Levels.HIGH;
                }
            } else if (op.getAlliance() == Alliance.RED) {
                RotatedRect boundingRect = tseDetector.getRect();
                if (boundingRect == null || boundingRect.size.area() < 1500) return Levels.HIGH;
                if (boundingRect.center.x <= 360 / 2.0) {
                    return Levels.LOW;
                } else {
                    return Levels.MIDDLE;
                }
            } else {
                return Levels.HIGH;
            }
        }
        catch(Exception e){
            if (op.getAlliance() == Alliance.BLUE)
                return Levels.LOW;
            else
                return Levels.HIGH;
        }
    }

    public void setLiftHeight(Lift lift, double height) {
        lift.setTargetHeight(height);
        op.customWait(() -> (lift.getHeight() < height - 0.5));
    }

    public void setLiftHeight(Lift lift, Levels level) {
        lift.setTargetHeight(level);
        op.customWait(() -> (lift.getHeight() < level.height - 0.5));
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
            spinner.setPower(-0.75);
            op.waitForTime(msTime);
        }
        else if (op.getAlliance() == Alliance.BLUE) {
            spinner.setPower(0.75);
            op.waitForTime(msTime);
        }

        spinner.setPower(0);
    }
}
