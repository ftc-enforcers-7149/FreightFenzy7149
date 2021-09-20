package org.firstinspires.ftc.teamcode.Odometry.Util;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

public class Pose2dConvert {

    public Pose2d convertToCamera(com.acmerobotics.roadrunner.geometry.Pose2d input) {

        return new Pose2d(input.getX(), input.getY(), new Rotation2d(input.getHeading()));

    }

    public com.acmerobotics.roadrunner.geometry.Pose2d convertToOdom(Pose2d input) {

        return new com.acmerobotics.roadrunner.geometry.Pose2d(input.getTranslation().getX(), input.getTranslation().getY(), input.getHeading());

    }

}
