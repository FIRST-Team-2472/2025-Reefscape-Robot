package frc.robot.subsystems;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.SensorStatus;

//import frc.robot.Constants.SensorStatus;
import frc.robot.LimelightHelpers.PoseEstimate;

public class LimeLightSubsystem extends SubsystemBase {

    private String[] LimeLights = { "limelight-two" };
    private double[] LimeLightConfidence = { 0 };
    private double[] LimeLightArea = { 0 };
    private double[] LimeLightDist = { 0 };
    private Pose2d[] LimeLightPose2d = { new Pose2d() };
    private double totalConfidence;

    public LimeLightSubsystem() {

    }

    public void addLimeLight(String name) {
        LimeLights[LimeLights.length] = name;
    }

    @Override
    public void periodic() {

        for (int i = 0; i < LimeLights.length; i++) {

            // Send Gyro data to Limelight for higher accuracy
            LimelightHelpers.SetRobotOrientation(LimeLights[i], SensorStatus.pigeonYaw,
            0.0, SensorStatus.pigeonPitch, 0.0, SensorStatus.pigeonRoll, 0.0);

            // Get the pose estimates
            PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(LimeLights[i]);

            System.out.println(estimate);

            if (estimate == null) {
                System.out.println("LimeLight " + LimeLights[i] + " not found");
                LimeLightConfidence[i] = 0;
                LimeLightArea[i] = 0;
                LimeLightDist[i] = 0;
                LimeLightPose2d[i] = new Pose2d();
                continue;
            }

            double targetArea = estimate.avgTagArea;
            double targetDistance = estimate.avgTagDist;

            // Confidence calculation
            double confidence = Math.min(((32.0f / (targetDistance + 4.0f)) - (0.25f * targetDistance) + (7.0f * targetArea)) * 0.1f + 0.02f, 1.0f);

            // Save the confidence
            LimeLightConfidence[i] = confidence;

            // Save the area
            LimeLightArea[i] = targetArea;

            // Save the distance
            LimeLightDist[i] = targetDistance;

            // Save the pose
            LimeLightPose2d[i] = estimate.pose;

            SmartDashboard.putNumber("AprilTagArea", targetArea);
            SmartDashboard.putNumber("AprilTagDist", targetDistance);
            SmartDashboard.putNumber("AprilTagConf", confidence);
        }
        
        System.out.println(Arrays.toString(getConfidences()));
        Constants.SensorStatus.LimeLightBotPoses = getBotPose2ds();
        Constants.SensorStatus.LimeLightConfidences = getConfidences();
    }

    public Pose2d getBotPose2d(int index) {
        return this.LimeLightPose2d[index];
    }

    public Pose2d[] getBotPose2ds() {
        return this.LimeLightPose2d;
    }

    public double getConfidence(int index) {
        return this.LimeLightConfidence[index];
    }

    public double[] getConfidences() {
        return this.LimeLightConfidence;
    }

    public double getArea(int index) {
        return this.LimeLightArea[index];
    }

    public double getDistance(int index) {
        return this.LimeLightDist[index];
    }

    public double getTotalConfidence() {
        return this.totalConfidence;
    }

}
