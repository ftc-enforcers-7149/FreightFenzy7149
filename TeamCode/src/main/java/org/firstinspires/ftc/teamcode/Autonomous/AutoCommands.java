package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.MotorCarouselSpinner;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs.MotorIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Levels;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Output;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam.OpenCV;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam.SimpleColorPipeline;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;

public class AutoCommands {

    private final Autonomous_Base op;

    public AutoCommands(Autonomous_Base op) {
        this.op = op;
    }

    public Levels detectBarcode(OpenCV tseDetector) {
        try {

            RotatedRect boundingRect = tseDetector.getRect();
            Rect rectangle = ((SimpleColorPipeline) tseDetector.getPipeline()).getCrop();

            if (boundingRect == null || boundingRect.size.area() < 1500) return Levels.LOW;
            if (boundingRect.center.x - rectangle.x <= rectangle.width / 2d) {
                return Levels.MIDDLE;
            } else {
                return Levels.HIGH;
            }

            /*if (op.getAlliance() == Alliance.BLUE) {
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
            }*/
        }
        catch(Exception e) {
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

    public void outtake(MotorIntake intake, Lift lift) {
        //if (lift.getHeight() <= (Levels.LOW.height + Levels.MIDDLE.height) / 2)
        //    intake.setIntakePower(0.6);
        intake.setLatch(MotorIntake.LatchPosition.OPEN);

        op.waitForTime(100);

        //if (lift.getHeight() <= (Levels.MIDDLE.height + Levels.HIGH.height) / 2)
            intake.setPaddle(MotorIntake.PaddlePosition.OUT_FAR);
        //else
        //    intake.setPaddle(MotorIntake.PaddlePosition.OUT_CLOSE);

        //if (lift.getHeight() <= Levels.MIDDLE.height)
        //    op.waitForTime(150);
        op.waitForTime(200);

        intake.setIntakePower(0);
        intake.setPaddle(MotorIntake.PaddlePosition.BACK);
    }

    public void outtake(MotorIntake intake) {
        intake.setLatch(MotorIntake.LatchPosition.OPEN);
        op.waitForTime(150);

        Output outtake = new Output() {
            long startTime = 0;

            @Override
            public void updateOutput() {
                if (System.currentTimeMillis() >= startTime + 150)
                    stopOutput();
            }

            @Override
            public void startOutput() {
                startTime = System.currentTimeMillis();
                intake.setPaddle(MotorIntake.PaddlePosition.OUT_FAR);
            }

            @Override
            public void stopOutput() {
                intake.setPaddle(MotorIntake.PaddlePosition.BACK);
                op.removeOutput(this);
            }
        };

        op.addOutput(outtake);
        outtake.startOutput();
    }

    public void closeIntake(MotorIntake intake) {
        intake.setIntakePower(0);
        intake.setPaddle(MotorIntake.PaddlePosition.BACK);
        intake.setLatch(MotorIntake.LatchPosition.CLOSED);

        op.waitForTime(150);
    }

    public void spinDuck(MotorCarouselSpinner spinner) {
        spinner.reset();
        op.customWait(spinner::isBusy);
    }
}
