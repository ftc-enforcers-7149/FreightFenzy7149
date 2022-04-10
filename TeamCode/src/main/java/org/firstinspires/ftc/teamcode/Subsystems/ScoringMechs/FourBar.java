package org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Utils.Interp;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Output;

public class FourBar implements Output {

    public Servo left, right;
    public Servo counterL, counterR;
    //private double currAngle, desiredAngle, lastDesiredAngle;
    private double currPos, desiredPos, lastPos;
    public boolean manualCounter;
    public double manualCounterPos, lastManualCounterPos;

    Interp interp;

    public FourBar(HardwareMap hardwareMap, String leftName, String rightName,
                   String counterLName, String counterRName) {
        left = hardwareMap.servo.get(leftName);
        left.setDirection(Servo.Direction.FORWARD);
        left.setPosition(scalePos(0));
        right = hardwareMap.servo.get(rightName);
        right.setDirection(Servo.Direction.REVERSE);
        right.setPosition(scalePos(0));

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
        currPos = interp.getValue();

        if (!manualCounter) {
            if (currPos != lastPos) {
                left.setPosition(scalePos(currPos));
                right.setPosition(scalePos(currPos));

                counterL.setPosition(currPos);
                counterR.setPosition(currPos);
            }
        }
        else {
            if (currPos != lastPos) {
                left.setPosition(scalePos(currPos));
                right.setPosition(scalePos(currPos));
            }
            if (manualCounterPos != lastManualCounterPos) {
                counterL.setPosition(manualCounterPos);
                counterR.setPosition(manualCounterPos);

                lastManualCounterPos = manualCounterPos;
            }
        }

        lastPos = currPos;
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
        interp = new Interp(currPos, desiredPos, 0, Interp.Method.INSTANT);
    }

    public void setPosition(double pos, double interpTime) {
        //goToAngle(pos * TRAVEL_DIST);
        desiredPos = pos;
        interp = new Interp(currPos, desiredPos, interpTime, Interp.Method.INSTANT);
    }

    public double getCurrAngle() {
        return currPos;
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
