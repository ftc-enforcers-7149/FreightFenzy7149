package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
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
    public static final double LEVEL_OFFSET = 2;
    public static final double CAP_OFFSET = -1.5;

    public static PIDCoefficients H_PID = new PIDCoefficients(-0.45, 0, -0.05);

    //How accurate each attribute should be at each point
    public static double POS_ACC = 1;
    public static double H_ACC = Math.toRadians(1);

    public static double SPEED_MULT = 1;

    public static double MIN_SPEED = 0.2;
    public static double MIN_TURN = 0.15;
    public static double CLOSE_DIST = 0;

    public static double SLOW_DIST = 15;
}
