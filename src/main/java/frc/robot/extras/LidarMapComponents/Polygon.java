package frc.robot.extras.LidarMapComponents;

public class Polygon {
    public LineSegment[] LineSegmentArray;
    public Polygon(LineSegment[] LineSegmentArray){
        this.LineSegmentArray = LineSegmentArray;
    }

    public LineSegment[] get(){
        return LineSegmentArray;
    }
}