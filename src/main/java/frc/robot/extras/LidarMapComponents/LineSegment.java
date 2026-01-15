package frc.robot.extras.LidarMapComponents;

public class LineSegment {
    public MapPoint startPoint, endPoint;
    public LineSegment(MapPoint startPoint, MapPoint endPoint){
        this.startPoint = startPoint;
        this.endPoint = endPoint;
    }
}
