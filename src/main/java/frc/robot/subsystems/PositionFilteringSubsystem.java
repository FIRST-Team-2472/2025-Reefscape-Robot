package frc.robot.subsystems;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PositionFilteringSubsystem extends SubsystemBase {

    private int numLimeLights = 1;

    private Pose2d filteredBotPose = new Pose2d();
    private Pose2d odometryBotPose = new Pose2d();
    private Pose2d[] limeLightBotPoses = new Pose2d[numLimeLights];

    private double odometryConfidence = 0.9d;
    private double[] limeLightConfidences = new double[numLimeLights];

    public PositionFilteringSubsystem() {

    }

    @Override
    public void periodic() {
        numLimeLights = Constants.SensorStatus.LimeLightBotPoses.length;
        
        double weightedX = 0;
        double weightedY = 0;

        odometryBotPose = Constants.SensorStatus.odometryBotPose;
        limeLightBotPoses = Constants.SensorStatus.LimeLightBotPoses;
        limeLightConfidences = Constants.SensorStatus.LimeLightConfidences;

        // System.out.println(limeLightBotPoses.length);

        for (int i = 0; i < numLimeLights; i++) {
            System.out.println("PositionFilteringSubsystem: Confidence: " + limeLightConfidences[i] + ", Pose: (" + limeLightBotPoses[i].getX() + ", " + limeLightBotPoses[i].getY() + ")");
        }

        if (numLimeLights == 0) {
            Constants.SensorStatus.filteredBotPose = odometryBotPose;
            return;
        }

        // An implementation of a super simple weighted average

        double[] confs = new double[numLimeLights + 1];

        // Copy the LimeLight confidences to entries 1-numLimeLights and the odometry
        // confidence to the 0th term
        System.arraycopy(limeLightConfidences, 0, confs, 1, numLimeLights - 1); // Maybe remove the -1?
        confs[0] = odometryConfidence;

        System.out.println("PositionFilteringSubsystem: Original Confidences: " + Arrays.toString(confs));

        // Now find the total confidence so we can normalize all the positions
        double totalConfidence = 0d;
        for (double conf : confs) {
            totalConfidence += conf;
        }

        System.out.println("PositionFilteringSubsystem: Total Confidence: " + totalConfidence);

        for (int i = 0; i < confs.length; i++) {
            confs[i] /= totalConfidence;
        }

        System.out.println("PositionFilteringSubsystem: Normalized Confidences: " + Arrays.toString(confs));

        // Multiply all positions to normalize them
        for (int i = 0; i < numLimeLights; i++) {
            weightedX += limeLightBotPoses[i].getX() * confs[i + 1];
            System.out.println("PositionFilteringSubsystem: LimeLightBotPose[" + i + "].X: " + limeLightBotPoses[i].getX() + ", Confidence: " + confs[i + 1]);
            weightedY += limeLightBotPoses[i].getY() * confs[i + 1];
            System.out.println("PositionFilteringSubsystem: LimeLightBotPose[" + i + "].Y: " + limeLightBotPoses[i].getY() + ", Confidence: " + confs[i + 1]);
        }
        
        weightedX += odometryBotPose.getX() * confs[0];
        weightedY += odometryBotPose.getY() * confs[0];

        System.out.println("PositionFilteringSubsystem: OdometryBotPose.X: " + odometryBotPose.getX() + ", Confidence: " + confs[0]);
        System.out.println("PositionFilteringSubsystem: OdometryBotPose.Y: " + odometryBotPose.getY() + ", Confidence: " + confs[0]);

        System.out.println("PositionFilteringSubsystem: Weighted X: " + weightedX);
        System.out.println("PositionFilteringSubsystem: Weighted Y: " + weightedY);

        filteredBotPose = new Pose2d(weightedX, weightedY, odometryBotPose.getRotation());

        Constants.SensorStatus.filteredBotPose = filteredBotPose;

        SmartDashboard.putNumber("Filtered Pose X", filteredBotPose.getX());
        SmartDashboard.putNumber("Filtered Pose Y", filteredBotPose.getY());
        SmartDashboard.putNumber("Total Confidence", totalConfidence);
        SmartDashboard.putNumber("numLimeLights", numLimeLights);
    }
}
