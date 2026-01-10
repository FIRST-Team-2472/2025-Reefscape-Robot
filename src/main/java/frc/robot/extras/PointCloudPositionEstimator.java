package frc.robot.extras;

import frc.robot.extras.LidarMapComponents.MapPoint;

public class PointCloudPositionEstimator {
    public static FieldPoint estimatePose(FieldPose2d roughPose, double[] pointCloud){
        MapPoint[] roughParticles = generateParticles(roughPose);
        return null;
    }

    
    public static MapPoint[] generateParticles(FieldPose2d roughPose){
        MapPoint[] roughParticles = new MapPoint[10];
        for(int i = 0; i < roughParticles.length; i++){
            roughParticles[i] = new MapPoint(
                roughPose.getX() + Math.random() * 0.05 - 0.025,// 5cm tolerance to start
                roughPose.getY() + Math.random() * 0.05 - 0.025
            );
            while(isTooClose(roughParticles[i], roughParticles, (short)i)){// regenerarte if too close to previous points
                roughParticles[i] = new MapPoint(
                    roughPose.getX() + Math.random() * 0.05 - 0.025,
                    roughPose.getY() + Math.random() * 0.05 - 0.025
                );
            }
        }
        return roughParticles;
    }
    public static boolean isTooClose(MapPoint pointToCheck, MapPoint[] PreviousPoints, short arrayPosition){
        for(int i = 0; i < arrayPosition; i++){
            double distance = Math.sqrt(
                (pointToCheck.x - PreviousPoints[i].x)*(pointToCheck.x - PreviousPoints[i].x) +
                (pointToCheck.y - PreviousPoints[i].y)*(pointToCheck.y - PreviousPoints[i].y)
            );
            if(distance < 0.01){ //5mm tolerance for duplicate points
                return true;
            }
        }
        return false;
    }
    public static void main(String[] args){

    }
}
