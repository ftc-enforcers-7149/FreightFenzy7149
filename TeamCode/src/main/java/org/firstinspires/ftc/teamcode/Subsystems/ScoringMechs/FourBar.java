package org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Hardware.BettaServo;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Output;

import static org.firstinspires.ftc.teamcode.GlobalData.SLIDE_ANGLE;

public class FourBar implements Output {

    public BettaServo left, right;
    public Servo counterL, counterR;
    private double currAngle, desiredAngle, lastDesiredAngle;
    public boolean manualCounter;
    public double manualCounterPos;

    public static final double TRAVEL_DIST = Math.toRadians(151); /*degrees*/

    public enum Position {
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

    }

    public FourBar(HardwareMap hardwareMap, String leftName, String rightName,
                   String counterLName, String counterRName) {
        left = new BettaServo(hardwareMap, leftName);
        left.servo.setDirection(Servo.Direction.FORWARD);
        right = new BettaServo(hardwareMap, rightName);
        right.servo.setDirection(Servo.Direction.REVERSE);

        left.setFullRangeTime(1000);
        right.setFullRangeTime(1000);

        counterL = hardwareMap.servo.get(counterLName);
        counterL.setDirection(Servo.Direction.REVERSE);
        counterR = hardwareMap.servo.get(counterRName);
        counterR.setDirection(Servo.Direction.FORWARD);

        goToAngle(Position.IN);
        updateOutput();
    }

    @Override
    public void updateOutput() {
        //Calculate angle from servo position, and servo positions from desired angle
        currAngle = left.getPosition() * TRAVEL_DIST - Math.PI / 2 - SLIDE_ANGLE;
        double desiredPos = scalePos((desiredAngle + Math.PI / 2 + SLIDE_ANGLE) / TRAVEL_DIST);
        double desiredCounterPos = Math.cos(desiredAngle * (Math.PI / (Math.PI + SLIDE_ANGLE / 2)));

        if (!manualCounter) {
            if (desiredAngle != lastDesiredAngle) {
                left.setPosition(desiredPos);
                right.setPosition(desiredPos);

                counterL.setPosition(desiredCounterPos);
                counterR.setPosition(desiredCounterPos);
            }

            lastDesiredAngle = desiredAngle;
        } else {
            if (desiredAngle != lastDesiredAngle) {
                left.setPosition(desiredPos);
                right.setPosition(desiredPos);

                counterL.setPosition(manualCounterPos);
                counterR.setPosition(manualCounterPos);
            }

            lastDesiredAngle = desiredAngle;
        }
    }

    public void goToAngle(double angle) {
        desiredAngle = Math.min(Math.max(-Math.PI/2-SLIDE_ANGLE, angle), -Math.PI/2-SLIDE_ANGLE + TRAVEL_DIST);
    }

    public void goToAngle(Position p) {
        goToAngle(p.angle);
    }

    public void goToServoPos(double pos) {
        goToAngle(pos * TRAVEL_DIST);
    }

    public double getCurrAngle() {
        return currAngle;
    }

    private double scalePos(double pos) {
        double zeroOutput = 0.06;
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
