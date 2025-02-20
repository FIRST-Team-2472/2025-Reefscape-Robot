package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;

public class SensorStatus {
    public static boolean kLeftLimitSwitchValue = false;
    public static boolean kMiddleLimitSwitchValue = false;
    public static boolean kRightLimitSwitchValue = false;

    public static double kElevatorHeight = 0;

    public static double pigeonYaw = 0;
    public static double pigeonPitch = 0;
    public static double pigeonRoll = 0;

    public static Pose2d odometryBotPose;

    public static Pose2d[] LimeLightBotPoses;
    public static double[] LimeLightConfidences;

    public static Pose2d filteredBotPose;

    public static double kTimeOfFlightDistance = 0;
    public static double kClimberAngle = 0;
    public static double kPivotAngle = 0;
}