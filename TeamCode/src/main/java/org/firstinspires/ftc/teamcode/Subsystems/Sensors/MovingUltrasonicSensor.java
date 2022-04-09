package org.firstinspires.ftc.teamcode.Subsystems.Sensors;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;

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
    private Gyroscope gyro;

    public MovingUltrasonicSensor(int smoothingSize, Facing f, Localizer localizer, Gyroscope gyro) {
        super(smoothingSize);
        this.f = f;
        this.localizer = localizer;
        this.gyro = gyro;
    }

    public double getTimeFromDistance(double distance) {

        return 1/SPEED_OF_SOUND * distance;

    }

    @Override
    public void add(double newVal) {

        Log.i("Adding: ", String.valueOf(newVal));

        double time = getTimeFromDistance(newVal);

        double heading = gyro.getRawYaw();
        double pi = Math.PI;

        if(heading >= pi/4 && heading <= 3*pi/4) {// facing "front" (positive y)
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
        else if(heading < pi/4 && heading >= 0 || heading > 7*pi/4 && heading <= 2*pi) {// facing "left" (positive x)
            switch(f) {

                case FRONT:
                    super.add(newVal  - localizer.getPoseVelocity().getX() * time);
                    break;
                case BACK:
                    super.add(newVal  + localizer.getPoseVelocity().getX() * time);
                    break;
                case LEFT:
                    super.add(newVal  + localizer.getPoseVelocity().getY() * time);
                    break;
                case RIGHT:
                    super.add(newVal  - localizer.getPoseVelocity().getY() * time);
                    break;

            }
        }
        else if(heading > 3*pi/4 && heading <= 5*pi/4) { // facing "right" (negative x)
            switch(f) {

                case FRONT:
                    super.add(newVal  + localizer.getPoseVelocity().getX() * time);
                    break;
                case BACK:
                    super.add(newVal  - localizer.getPoseVelocity().getX() * time);
                    break;
                case LEFT:
                    super.add(newVal  - localizer.getPoseVelocity().getY() * time);
                    break;
                case RIGHT:
                    super.add(newVal  + localizer.getPoseVelocity().getY() * time);
                    break;

            }
        }
        else { // facing "back" (negative y)
            switch(f) {

                case FRONT:
                    super.add(newVal  + localizer.getPoseVelocity().getY() * time);
                    break;
                case BACK:
                    super.add(newVal  - localizer.getPoseVelocity().getY() * time);
                    break;
                case LEFT:
                    super.add(newVal  - localizer.getPoseVelocity().getX() * time);
                    break;
                case RIGHT:
                    super.add(newVal  + localizer.getPoseVelocity().getX() * time);
                    break;

            }
        }

    }
}
