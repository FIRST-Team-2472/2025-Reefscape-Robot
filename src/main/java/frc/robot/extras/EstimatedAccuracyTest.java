package frc.robot.extras;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.extras.LidarMapComponents.LidarSimulator;

public class EstimatedAccuracyTest {
    public EstimatedAccuracyTest() {
        // This class is currently a placeholder for future implementation
    }
    public static void Main(String[] args) {
        for(int i = 0; i < 10; i++) {
            double trueX = Math.random() * 10;
            double trueY = Math.random() * 10;
            double trueRotation = Math.random() * 360;

            double maxTranslationalInaccuracy  =0.03; // 3 cm
            double maxAngleInaccuracy = 3; // 3 degrees

            double innacurateX = trueX + (Math.random() * 2 - 1) * maxTranslationalInaccuracy;
            double innacurateY = trueY + (Math.random() * 2 - 1) * maxTranslationalInaccuracy;
            double innacurateRotation = trueRotation + (Math.random() * 2 - 1) * maxAngleInaccuracy;

            double[] SimulatedScan = LidarSimulator.getMessySimData(
                new frc.robot.extras.LidarMapComponents.MapPoint(trueX, trueY),
                trueRotation
            );
            FieldPoint calculatedPose = PointCloudPositionEstimator.estimatePose(new FieldPose2d(innacurateX, innacurateY, new Rotation2d().fromDegrees(innacurateRotation)), SimulatedScan);
            double accuracyImprovement = Math.hypot(trueX - calculatedPose.getX(), trueY - calculatedPose.getY()) - Math.hypot(trueX - innacurateX, trueY - innacurateY);
            System.out.println("accuracy Imporvement = " + accuracyImprovement);
        }
    }
}
