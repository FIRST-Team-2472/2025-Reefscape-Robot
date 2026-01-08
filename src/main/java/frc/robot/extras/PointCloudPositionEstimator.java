package frc.robot.extras;

import frc.robot.extras.LidarMapComponents.MapPoint;

public class PointCloudPositionEstimator {
    public static FieldPoint estimatePose(FieldPose2d roughPose, double[] pointCloud){
        MapPoint[] roughParticles = new MapPoint[10];
        for(int i = 0; i < roughParticles.length; i++){
            roughParticles[i] = new MapPoint(
                roughPose.getX() + Math.random() * 0.05 - 0.025,// 5cm tolerance to start
                roughPose.getY() + Math.random() * 0.05 - 0.025
            );

        }
        return null;
    }
    public static boolean isTooClose(MapPoint )
    public static void main(String[] args){

    }
}
