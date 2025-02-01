package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PositionFilteringSubsystem extends SubsystemBase {

    private int numLimeLights = Constants.SensorStatus.LimeLightBotPoses.length;

    private Pose2d filteredBotPose = new Pose2d();
    private Pose2d odometryBotPose = new Pose2d();
    private Pose2d[] limeLightBotPoses = new Pose2d[numLimeLights];

    private double odometryConfidence = 0.9d;
    private double[] limeLightConfidences = new double[numLimeLights];

    public PositionFilteringSubsystem() {

    }

    @Override
    public void periodic() {
        odometryBotPose = Constants.SensorStatus.odometryBotPose;
        limeLightBotPoses = Constants.SensorStatus.LimeLightBotPoses;

        // An implementation of a super simple weighted average

        double[] confs = new double[numLimeLights + 1];

        // Copy the LimeLight confidences to entries 1-numLimeLights and the odometry
        // confidence to the 0th term
        System.arraycopy(limeLightConfidences, 0, confs, 1, numLimeLights);
        confs[0] = odometryConfidence;

        // Now find the total confidence so we can normalize all the positions
        double totalConfidence = 0d;
        for (double conf : confs) {
            totalConfidence += conf;
        }

        for (int i = 0; i < confs.length; i++) {
            confs[i] /= totalConfidence;
        } 

        Pose2d weightedPose2d;

        // Divide all positions to normalize them
        for (int i = 0; i < numLimeLights; i++) {
            weightedPose2d = limeLightBotPoses[i].times(confs[i + 1]);

            filteredBotPose = filteredBotPose.plus(
                    new Transform2d(weightedPose2d.getTranslation(), weightedPose2d.getRotation()));
        }
        
        weightedPose2d = odometryBotPose.times(confs[0]);
        filteredBotPose = filteredBotPose.plus(
                    new Transform2d(weightedPose2d.getTranslation(), weightedPose2d.getRotation()));

        Constants.SensorStatus.filteredBotPose = filteredBotPose;
    }
}
