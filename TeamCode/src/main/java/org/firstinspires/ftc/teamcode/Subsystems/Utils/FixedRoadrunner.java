package org.firstinspires.ftc.teamcode.Subsystems.Utils;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class FixedRoadrunner {

    public static Pose2d createPose2d(double x, double y, double heading) {
        return new Pose2d(y, -x, Math.toRadians(heading));
    }

    public static Vector2d createVector2d(double x, double y) {
        return new Vector2d(y, -x);
    }
}
