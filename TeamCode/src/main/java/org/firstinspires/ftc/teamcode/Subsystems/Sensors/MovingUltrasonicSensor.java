package org.firstinspires.ftc.teamcode.Subsystems.Sensors;

import com.acmerobotics.roadrunner.localization.Localizer;

import java.util.ArrayList;

public class MovingUltrasonicSensor extends Sensor {

    public enum Facing {

        FRONT,
        BACK,
        LEFT,
        RIGHT

    }

    public static final double SPEED_OF_SOUND = 343 /*m/s, in air at 68F*/;

    private Localizer localizer;
    private Facing f;

    public MovingUltrasonicSensor(int smoothingSize, Facing f, Localizer localizer) {
        super(smoothingSize);
        this.f = f;
        this.localizer = localizer;
    }

    public double getTimeFromDistance(double distance) {

        return 1/SPEED_OF_SOUND * distance;

    }

    @Override
    public void add(double newVal) {

        double time = getTimeFromDistance(newVal);

        switch(f) {

            case FRONT:
                super.add(newVal  - localizer.getPoseVelocity().getY() * time);
                break;
            case BACK:
                super.add(newVal  + localizer.getPoseVelocity().getY() * time);
                break;
            case LEFT:
                super.add(newVal  + localizer.getPoseVelocity().getX() * time);
                break;
            case RIGHT:
                super.add(newVal  - localizer.getPoseVelocity().getX() * time);
                break;

        }

    }
}
