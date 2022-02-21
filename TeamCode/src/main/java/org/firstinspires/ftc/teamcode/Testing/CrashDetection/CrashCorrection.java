package org.firstinspires.ftc.teamcode.Testing.CrashDetection;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors.Gyroscope;

public class CrashCorrection {

    private Drive d;
    private Pose2d target, startPos;
    private Gyroscope g;

    private static double ACCEL_THRESHOLD = 0.05; // TODO this is arbitrary test/fix it

    public CrashCorrection(Drive d, Gyroscope g, Pose2d target) {
        this.d = d;
        this.g = g;
        this.target = target;
        this.startPos = d.getPoseEstimate();
    }

    public Pose2d correct(Pose2d weightedPowers) {

        Acceleration a = g.imu.getAcceleration();

        // this is just a 3d distance formula. you did this late at night so check it again
        double avgAccel = Math.sqrt(Math.pow(a.xAccel, 2) + Math.pow(a.zAccel, 2) + Math.pow(a.yAccel, 2));

        return avgAccel >= ACCEL_THRESHOLD ? weightedPowers : /*todo make up some bullshit doohickery correction*/
            new Pose2d(0,0,0);

    }

}
