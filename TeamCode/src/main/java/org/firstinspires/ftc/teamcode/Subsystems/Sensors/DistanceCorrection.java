package org.firstinspires.ftc.teamcode.Subsystems.Sensors;

import android.renderscript.ScriptGroup;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Odometry.DriveWheels.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Input;

public class DistanceCorrection implements Input {

    private DistanceSensor distL, distR, distF;
    private double lDist, rDist, fDist;
    private static double FIELD_X = 144, FIELD_Y = 144, F_OFFSET = 6.75 - 2.65, L_R_OFFSET = 7.5 - 0.3;

    public DistanceCorrection(HardwareMap hardwareMap, String distLName, String distRName, String distFName) {

        distL = hardwareMap.get(DistanceSensor.class, distLName);
        distR = hardwareMap.get(DistanceSensor.class, distRName);
        distF = hardwareMap.get(DistanceSensor.class, distFName);

    }

    public void correctPoseWithDist(MecanumDrive drive, Alliance alliance) {
        Pose2d drivePose = drive.getPoseEstimate();

        double distX = alliance.equals(Alliance.BLUE) ? lDist : rDist;
        double distY = (FIELD_Y - fDist) * (alliance.equals(Alliance.BLUE) ? 1 : -1);

        drive.setPoseEstimate(new Pose2d(distX, distY, drivePose.getHeading()));

    }

    @Override
    public void updateInput() {
        lDist = distL.getDistance(DistanceUnit.INCH) + L_R_OFFSET;
        rDist = distR.getDistance(DistanceUnit.INCH) + L_R_OFFSET;
        fDist = distF.getDistance(DistanceUnit.INCH) + F_OFFSET;
    }

}
