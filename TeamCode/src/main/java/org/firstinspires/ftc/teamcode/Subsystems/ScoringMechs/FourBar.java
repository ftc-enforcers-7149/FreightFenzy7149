package org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Utils.Input;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Output;

public class FourBar implements Output, Input {

    private Servo left, right, counterL, counterR;
    private double angle, desiredAngle, counterAngle, desiredCounterAngle;
    private boolean atPos = true, atCounterPos = true;

    public static final double TRAVEL_DIST = 151 /*degrees*/;

    public enum Position {

        IN(0),
        HALF(56),
        OUT(113),
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

    public FourBar(HardwareMap hardwareMap, String leftName, String rightName) {
        left = hardwareMap.servo.get(leftName);
        left.setDirection(Servo.Direction.FORWARD);
        right = hardwareMap.servo.get(rightName);
        right.setDirection(Servo.Direction.REVERSE);
    }

    public void init() {

        // todo move to zero positions

    }

    @Override
    public void updateInput() {

        angle = (left.getPosition() + right.getPosition()) / 2;
        counterAngle = (counterL.getPosition() + counterR.getPosition()) / 2;

        desiredCounterAngle = Math.cos(Math.toRadians(angle));

    }

    @Override
    public void updateOutput() {

        if(angle != desiredAngle) {

            atPos = false;
            left.setPosition(scalePos(angle));
            right.setPosition(scalePos(angle));

        }
        else atPos = true;

        if(counterAngle != desiredCounterAngle) {

            atPos = false;
            left.setPosition(scalePos(counterAngle));
            right.setPosition(scalePos(counterAngle));

        }
        else atCounterPos = true;

    }

    public double getAngle() { return angle; }
    public boolean isAtPos() { return atPos; }
    public boolean isAtCounterPos() { return atCounterPos; }

    public void goToAngle(double angle) {

        desiredAngle = angle;

    }

    public void goToAngle(Position p) {

        desiredAngle = p.angle;

    }

    private double scalePos(double pos) {
        double zeroOutput = 0.06;
        double oneOutput = 1;

        return (oneOutput-zeroOutput) * (pos / TRAVEL_DIST)  + zeroOutput;
    }

}
