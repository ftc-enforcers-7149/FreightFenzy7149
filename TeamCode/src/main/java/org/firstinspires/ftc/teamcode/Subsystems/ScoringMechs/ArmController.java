package org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs;

import org.firstinspires.ftc.teamcode.Subsystems.Utils.Output;

public class ArmController implements Output {

    public final Lift lift;
    public final FourBar fourBar;

    public enum ScoringPosition {
        IDLE(0, 0, 1),
        IN(0, 0, 1),
        UP(7, 0, 1),
        LOW(0, 0.5, 0.6),
        MIDDLE(0, 0.75, 0.6),
        HIGH(4.8, 0.815, 0.6),
        CLOSE(5, 0, 0.6),
        CENTER(4.5, 0.2, 0.6),
        FAR(3.5, 0.35, 0.6),
        REACH(0, 0.6, 0.45);

        public double liftPos, barPos, maxSpeed;

        ScoringPosition(double liftPos, double barPos, double maxSpeed) {
            this.liftPos = liftPos;
            this.barPos = barPos;
            this.maxSpeed = maxSpeed;
        }
    }
    private ScoringPosition lastScorePos = ScoringPosition.IN;

    public ArmController(Lift lift, FourBar fourBar) {
        this.lift = lift;
        this.fourBar = fourBar;
    }

    public void setScorePos(ScoringPosition scorePos) {
        if (scorePos != lastScorePos) {
            lift.setTargetHeight(scorePos.liftPos, scorePos.maxSpeed);
            fourBar.setPosition(scorePos.barPos);

            lastScorePos = scorePos;
        }
    }

    /*public ArmController(Lift lift, FourBar fourBar) {
        this.lift = lift;
        this.fourBar = fourBar;

        currPos = new Vector2d(0, 0);
        destPos = new Vector2d(0, 0);
        lastDestPos = new Vector2d(0, 0);

        cosSA = Math.cos(SLIDE_ANGLE);
        sinSA = Math.cos(SLIDE_ANGLE);
        barLength = BAR_LENGTH;
    }

    @Override
    public void updateOutput() {
        currPos = calculatePosition();

        if (destPos != lastDestPos) {
            lift.setTargetHeight(getSlideLength(destPos));
            fourBar.goToAngle(getBarAngle(destPos));
        }

        lastDestPos = destPos;
    }

    public void setPosition(Vector2d pos) {
        destPos = pos;
    }

    public Vector2d getPosition() {
        return currPos;
    }

    private double getSlideLength(Vector2d pos) {
        return Math.sqrt(Math.pow(pos.getX() * cosSA + pos.getY() * sinSA, 2) +
                Math.pow(barLength, 2) - Math.pow(pos.getX(), 2) - Math.pow(pos.getY(), 2)) +
                pos.getX() * cosSA + pos.getY() * sinSA;
    }

    private double getBarAngle(Vector2d pos) {
        return -Math.acos((pos.getX() - cosSA * (
                Math.sqrt(Math.pow(pos.getX() * cosSA + pos.getY() * sinSA, 2) +
                        Math.pow(barLength, 2) - Math.pow(pos.getX(), 2) - Math.pow(pos.getY(), 2)) +
                        pos.getX() * cosSA + pos.getY() * sinSA
        )) / barLength);
    }

    private double getSlideSpeed(Vector2d vel) {
        return (
                (currPos.getX() * cosSA + currPos.getY() * sinSA) * (vel.getX() * cosSA + vel.getY() * sinSA) -
                        currPos.getX() * vel.getX() - currPos.getY() * vel.getY()
        ) /
                (
                        Math.sqrt(Math.pow(currPos.getX() * cosSA + currPos.getY() * sinSA, 2) +
                                Math.pow(barLength, 2) - Math.pow(currPos.getX(), 2) - Math.pow(currPos.getY(), 2))
                ) +
                vel.getX() * cosSA + vel.getY() * sinSA;
    }

    private double getBarAngSpeed(Vector2d vel) {
        return -(vel.getX() - cosSA * getSlideSpeed(new Vector2d(vel.getX(), vel.getY()))) /
                (barLength * Math.sin(getBarAngle(new Vector2d(currPos.getX(), currPos.getY()))));
    }

    private Vector2d calculatePosition() {
        return new Vector2d(
                lift.getHeight() * cosSA + barLength * Math.cos(fourBar.getCurrAngle()),
                lift.getHeight() * sinSA + barLength * Math.sin(fourBar.getCurrAngle())
        );
    }*/
}
