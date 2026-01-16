package frc.robot.extras;

import org.junit.jupiter.api.Test;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.extras.LidarMapComponents.LidarSimulator;

public class EstimatedAccuracyTest {

    @Test
    public void estimateTest() {
        int iterations = 300;
        int interationsWhereUsed = 0;
        double totalImprovement = 1;
        for(int i = 0; i < iterations; i++) {
            double trueX = Math.random() * 10;
            double trueY = Math.random() * 10;
            double trueRotation = Math.random() * 360;

            double maxTranslationalInaccuracy  = 0.03; // 3 cm
            double maxAngleInaccuracy = 3; // 3 degrees

            double innacurateX = trueX + (Math.random() * 2 - 1) * maxTranslationalInaccuracy;
            double innacurateY = trueY + (Math.random() * 2 - 1) * maxTranslationalInaccuracy;
            double innacurateRotation = trueRotation + (Math.random() * 2 - 1) * maxAngleInaccuracy;

            //currently set to lidar with messy data
            double[] SimulatedScan = LidarSimulator.getMessySimData(
                new frc.robot.extras.LidarMapComponents.MapPoint(trueX, trueY),
                trueRotation
            );
            FieldPoint calculatedPose = PointCloudPositionEstimator.estimatePose(new FieldPose2d(innacurateX, innacurateY, new Rotation2d().fromDegrees(innacurateRotation)), SimulatedScan);
            double lidarInnacuracy = Math.hypot(trueX - calculatedPose.getX(), trueY - calculatedPose.getY());
            double inputInnacuracy = Math.hypot(trueX - innacurateX, trueY - innacurateY);
            if((inputInnacuracy - lidarInnacuracy) != 0)
                System.out.println("accuracy Imporvement = " + (inputInnacuracy - lidarInnacuracy) + " lidar innac = " + lidarInnacuracy + " previous innac = " + inputInnacuracy);
            totalImprovement += (inputInnacuracy - lidarInnacuracy);
            if((inputInnacuracy - lidarInnacuracy) != 0)
                interationsWhereUsed++;
        }
        System.out.println("Average improvement over " + interationsWhereUsed + " useful iterations: " + (totalImprovement / interationsWhereUsed) + " meters. & total = " + totalImprovement);
    }
}
