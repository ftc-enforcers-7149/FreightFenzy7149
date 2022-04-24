package org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs;

import org.firstinspires.ftc.teamcode.Subsystems.Utils.Output;

public class ArmController implements Output {

    public final Lift lift;
    public final FourBar fourBar;

    public enum ScoringPosition {
        IDLE(7, 0, 1, 500),
        IN(0, 0, 1, 0),
        UP(7, 0, 1, 500),
        LOW(1.5, 0.55, 0.6, 0),
        MIDDLE(0.2, 0.8, 0.3, 500),
        HIGH(5, 0.815, 0.3, 0),
        HIGH_ARM(0.2, 1, 0.3, 0),
        CLOSE(5, 0, 0.6, 0),
        CENTER(5.25, 0.2, 0.45, 0),
        FAR(4.75, 0.31, 0.45, 0),
        REACH(0, 0.6, 0.45, 0),
        HIGH_AUTO(5.5, 0.815, 0.4, 0),
        MIDDLE_AUTO(1.5, 0.815, 0.1, 0),
        LOW_AUTO(1.5, 0.65, 0.1, 0),
        PARTIAL_UP(3.5, 0, 1, 500),
        CAP_PICKUP(1.65, 0.55, 0.45, 0);

        public double liftPos, barPos, maxSpeed, interpTime;

        ScoringPosition(double liftPos, double barPos, double maxSpeed, double interpTime) {
            this.liftPos = liftPos;
            this.barPos = barPos;
            this.maxSpeed = maxSpeed;
            this.interpTime = interpTime;
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
            fourBar.setPosition(scorePos.barPos, scorePos.interpTime);

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
