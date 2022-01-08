package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.Auto_V2;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors.DistanceCorrection;

@Autonomous(name = "Distance Correction Test")
public class DistanceCorrectionTest extends Auto_V2 {

    @Override
    public void auto() {
        while(opModeIsActive()) {
            updateInputs();

            drive.setPoseEstimate(distCorrect.correctPoseWithDist());
            telemetry.addData("Pose x:", drive.getPoseEstimate().getX());
            telemetry.addData("Pose y:", drive.getPoseEstimate().getY());

            updateOutputs();
        }
    }

    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }
}