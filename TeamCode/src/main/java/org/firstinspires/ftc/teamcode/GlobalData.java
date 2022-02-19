package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;

@Config
public class GlobalData {
    //Match Data
    public static Alliance ALLIANCE = Alliance.NONE;
    public static boolean RAN_AUTO = false;
    public static long MATCH_START_TIME = 0;
    public static long MATCH_END_TIME() { return MATCH_START_TIME + 128000; }

    //Drivetrain Data
    public static Pose2d POSITION = new Pose2d();
    public static double HEADING = 0;

    //Lift Data
    public static final double LEVEL_OFFSET = 2.5;
    public static final double CAP_OFFSET = -2;
}
