package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;

public class RobotStatus {
    public static boolean kLeftLimitSwitchValue = false;
    public static boolean kMiddleLimitSwitchValue = false;
    public static boolean kRightLimitSwitchValue = false;
    public static boolean kidMode = false;
    public static boolean elevatorSafety = true;

    public static double kElevatorHeight = 0;

    public static double pigeonYaw = 0;
    public static double pigeonPitch = 0;
    public static double pigeonRoll = 0;

    public static Pose2d odometryBotPose;

    public static Pose2d[] LimeLightBotPoses;
    public static double[] LimeLightConfidences;

    public static Pose2d filteredBotPose;

    public static double kTimeOfFlightDistance = -1;
    public static double kClimberAngle = 0;
    public static double kPivotAngle = 0;

    public static boolean seeCoral = false;
    public static boolean hasCoral = false;

    public static boolean isDispensing = false;
}