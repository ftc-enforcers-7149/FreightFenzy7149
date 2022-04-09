package org.firstinspires.ftc.teamcode.Subsystems.Sensors;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;

import org.firstinspires.ftc.teamcode.Autonomous.Autonomous_Base;

import java.util.ArrayList;

public class MovingUltrasonicSensor extends Sensor {

    public enum Facing {
        FRONT,
        BACK,
        LEFT,
        RIGHT
    }

    public static final double SPEED_OF_SOUND = 13503.9 /*in/s, in air at 68F*/;

    private Localizer localizer;
    private Facing f;

    private boolean disabled;

    public MovingUltrasonicSensor(int smoothingSize, Facing f, Localizer localizer) {
        super(smoothingSize);
        this.f = f;
        this.localizer = localizer;
    }

    public double getTimeFromDistance(double distance) {

        return 1/SPEED_OF_SOUND * distance;

    }

    public void disableMoving() {
        disabled = true;
    }

    public void enableMoving() {
        disabled = false;
    }

    @Override
    public void add(double newVal) {

        Log.i("Adding: ", String.valueOf(newVal));

        double time = getTimeFromDistance(newVal);

        double heading = localizer.getPoseEstimate().getHeading();
        double pi = Math.PI;

        if (!disabled) {
            if (Autonomous_Base.deltaHeading(localizer.getPoseEstimate().getHeading(), Math.toRadians(90)) <= Math.toRadians(45)) {
                switch (f) {
                    case FRONT:
                        super.add(newVal - localizer.getPoseVelocity().getY() * time);
                        break;
                    case BACK:
                        super.add(newVal + localizer.getPoseVelocity().getY() * time);
                        break;
                    case LEFT:
                        super.add(newVal + localizer.getPoseVelocity().getX() * time);
                        break;
                    case RIGHT:
                        super.add(newVal - localizer.getPoseVelocity().getX() * time);
                        break;
                }
            }
            else if (Autonomous_Base.deltaHeading(localizer.getPoseEstimate().getHeading(), Math.toRadians(-90)) <= Math.toRadians(45)){
                switch (f) {
                    case FRONT:
                        super.add(newVal + localizer.getPoseVelocity().getY() * time);
                        break;
                    case BACK:
                        super.add(newVal - localizer.getPoseVelocity().getY() * time);
                        break;
                    case LEFT:
                        super.add(newVal - localizer.getPoseVelocity().getX() * time);
                        break;
                    case RIGHT:
                        super.add(newVal + localizer.getPoseVelocity().getX() * time);
                        break;
                }
            }
            else
                super.add(newVal);
        }
        else
            super.add(newVal);
    }
}
