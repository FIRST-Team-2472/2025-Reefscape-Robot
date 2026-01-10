package frc.robot.extras;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.extras.LidarMapComponents.LidarSimulator;
import frc.robot.extras.LidarMapComponents.MapPoint;

//this class is designed to take in a point cloud and a rough pose estimate and return a refined pose estimate
//it does so by comparing the measured point cloud to simulated point clouds at various nearby positions and selecting the best match
//it does not however improve rotational accuracy yet

public class PointCloudPositionEstimator {
    public static FieldPoint estimatePose(FieldPose2d roughPose, double[] measuredPointCloud) {
        MapPoint robotPose = new MapPoint(roughPose.getX(), roughPose.getY());
        MapPoint[] roughParticles = generateParticles(robotPose, 10, 0.05, 0.01);
        double[] particleScores = scoreParticles(roughParticles, measuredPointCloud,
                roughPose.getRotation().getDegrees());// lower is better
        MapPoint bestParticle = getBestParticle(roughParticles, particleScores);
        // we have the 1st estimate now we need to refine this
        MapPoint[] moderateParticles = generateParticles(bestParticle, 10, 0.02, 0.003);
        double[] moderateParticleScores = scoreParticles(moderateParticles, measuredPointCloud,
                roughPose.getRotation().getDegrees());
        MapPoint bestModerateParticle = getBestParticle(moderateParticles, moderateParticleScores);

        return new FieldPoint(bestModerateParticle.x, bestModerateParticle.y);
    }

    public static MapPoint getBestParticle(MapPoint[] particles, double[] particleScores) {
        double bestScore = Double.MAX_VALUE;
        int bestIndex = -1;
        for (int i = 0; i < particleScores.length; i++) {
            if (particleScores[i] < bestScore) {
                bestScore = particleScores[i];
                bestIndex = i;
            }
        }
        SmartDashboard.putNumber("BestScore", bestScore);
        return particles[bestIndex];
    }

    public static double[] scoreParticles(MapPoint[] roughParticles, double[] measuredPointCloud,
            double robotAngleDegrees) {
        double[] particleScores = new double[roughParticles.length];// lower is better
        for (int i = 0; i < roughParticles.length; i++) {// we just assume rotation is perfect for now
            double[] expectedMeasurementAtParticle = LidarSimulator.getPerfectSimData(roughParticles[i],
                    robotAngleDegrees);
            double score = 0;
            for (int j = 0; j < measuredPointCloud.length; j++) {
                score += (measuredPointCloud[j] - expectedMeasurementAtParticle[j])
                        * (measuredPointCloud[j] - expectedMeasurementAtParticle[j]);
            }
            particleScores[i] = score;
        }
        return particleScores;
    }

    public static MapPoint[] generateParticles(MapPoint estimatedPose, int count, double maxSpread, double minSpread) {
        MapPoint[] roughParticles = new MapPoint[count];
        roughParticles[roughParticles.length - 1] = estimatedPose; // add the rough pose as a particle so we dont
                                                                   // decrease in accuracy
        for (int i = 0; i < roughParticles.length - 1; i++) {
            roughParticles[i] = new MapPoint(
                    estimatedPose.x + Math.random() * maxSpread - maxSpread / 2,
                    estimatedPose.x + Math.random() * maxSpread - maxSpread / 2);
            int tooCloseCount = 0;
            while (isTooClose(roughParticles[i], roughParticles, (short) i, minSpread) && tooCloseCount < 4) {// regenerarte
                                                                                                              // if too
                                                                                                              // close
                                                                                                              // to
                                                                                                              // previous
                                                                                                              // points
                tooCloseCount++;
                roughParticles[i] = new MapPoint(
                        estimatedPose.x + Math.random() * maxSpread - maxSpread / 2,
                        estimatedPose.x + Math.random() * maxSpread - maxSpread / 2);
            }
        }
        return roughParticles;
    }

    public static boolean isTooClose(MapPoint pointToCheck, MapPoint[] PreviousPoints, short arrayPosition,
            double minDistance) {
        for (int i = 0; i < arrayPosition; i++) {
            double distance = Math.sqrt(
                    (pointToCheck.x - PreviousPoints[i].x) * (pointToCheck.x - PreviousPoints[i].x) +
                            (pointToCheck.y - PreviousPoints[i].y) * (pointToCheck.y - PreviousPoints[i].y));
            if (distance < minDistance) { // 1mm tolerance for duplicate points
                return true;
            }
        }
        return false;
    }

    public static void main(String[] args) {

    }
}
