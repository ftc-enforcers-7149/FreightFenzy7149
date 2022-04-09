package org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Utils.Output;

public class FourBar implements Output {

    public Servo left, right;
    public Servo counterL, counterR;
    //private double currAngle, desiredAngle, lastDesiredAngle;
    private double currPos, desiredPos, lastDesiredPos;
    public boolean manualCounter;
    public double manualCounterPos, lastManualCounterPos;

    //public static final double TRAVEL_DIST = Math.toRadians(151); /*degrees*/

    /*public enum Position {
        IN(Math.toRadians(-107)),
        HALF(Math.toRadians(-60)),
        OUT(Math.toRadians(0)),
        MAX(TRAVEL_DIST);

        private final double angle;

        Position(double angle) { this.angle = angle; }

        public Position goUp() {
            if (this == MAX) return MAX;
            return values()[this.ordinal() + 1];
        }

        public Position goDown() {
            if (this == IN) return IN;
            return values()[this.ordinal() - 1];
        }

    }*/

    public FourBar(HardwareMap hardwareMap, String leftName, String rightName,
                   String counterLName, String counterRName) {
        left = hardwareMap.servo.get(leftName);
        left.setDirection(Servo.Direction.FORWARD);
        left.setPosition(0);
        right = hardwareMap.servo.get(rightName);
        right.setDirection(Servo.Direction.REVERSE);
        right.setPosition(0);

        //left.setFullRangeTime(1000);
        //right.setFullRangeTime(1000);

        counterL = hardwareMap.servo.get(counterLName);
        counterL.setDirection(Servo.Direction.REVERSE);
        counterR = hardwareMap.servo.get(counterRName);
        counterR.setDirection(Servo.Direction.FORWARD);

        //goToAngle(Position.IN);
        setPosition(0);
        updateOutput();
    }

    @Override
    public void updateOutput() {
        //Calculate angle from servo position, and servo positions from desired angle
        //currAngle = left.getPosition() * TRAVEL_DIST - Math.PI / 2 - SLIDE_ANGLE;
        //double desiredPos = scalePos((desiredAngle + Math.PI / 2 + SLIDE_ANGLE) / TRAVEL_DIST);
        //double desiredCounterPos = Math.cos(desiredAngle * (Math.PI / (Math.PI + SLIDE_ANGLE / 2)));

        if (!manualCounter) {
            //if (desiredPos != lastDesiredPos) {
                left.setPosition(scalePos(desiredPos));
                right.setPosition(scalePos(desiredPos));

                counterL.setPosition(desiredPos);
                counterR.setPosition(desiredPos);
            //}
        }
        else {
            if (desiredPos != lastDesiredPos) {
                left.setPosition(scalePos(desiredPos));
                right.setPosition(scalePos(desiredPos));
            }
            if (manualCounterPos != lastManualCounterPos) {
                counterL.setPosition(manualCounterPos);
                counterR.setPosition(manualCounterPos);

                lastManualCounterPos = manualCounterPos;
            }
        }

        lastDesiredPos = desiredPos;
    }

    /*public void goToAngle(double angle) {
        desiredAngle = Math.min(Math.max(-Math.PI/2-SLIDE_ANGLE, angle), -Math.PI/2-SLIDE_ANGLE + TRAVEL_DIST);
    }

    public void goToAngle(Position p) {
        goToAngle(p.angle);
    }*/

    public void setPosition(double pos) {
        //goToAngle(pos * TRAVEL_DIST);
        desiredPos = pos;
    }

    public double getCurrAngle() {
        return desiredPos;
    }

    private double scalePos(double pos) {
        double zeroOutput = 0.05;
        double oneOutput = 1;
        return (oneOutput-zeroOutput) * pos  + zeroOutput;
    }

    public void setManualCounterPos(double pos) {
        manualCounter = true;
        manualCounterPos = pos;
    }

    public void stopManualCounter() {
        manualCounter = false;
    }
}
