package frc.robot.extras;

import frc.robot.extras.LidarMapComponents.Polygon;

public class LidarMap {
    public static Polygon[] map = new Polygon[2];

    public LidarMap() {
        //all numbers are magic so far
        // Define polygons and add them to the map array
        // Polygon 1 aka the field walls 
        //currently just a 10x10 square
    }
    public static Polygon[] getMap() {
        map[0] = new Polygon(new frc.robot.extras.LidarMapComponents.LineSegment[] {
                new frc.robot.extras.LidarMapComponents.LineSegment(
                        new frc.robot.extras.LidarMapComponents.MapPoint(0, 0),
                        new frc.robot.extras.LidarMapComponents.MapPoint(10, 0)),
                new frc.robot.extras.LidarMapComponents.LineSegment(
                        new frc.robot.extras.LidarMapComponents.MapPoint(10, 0),
                        new frc.robot.extras.LidarMapComponents.MapPoint(10, 10)),
                new frc.robot.extras.LidarMapComponents.LineSegment(
                        new frc.robot.extras.LidarMapComponents.MapPoint(10, 10),
                        new frc.robot.extras.LidarMapComponents.MapPoint(0, 10)),
                new frc.robot.extras.LidarMapComponents.LineSegment(
                        new frc.robot.extras.LidarMapComponents.MapPoint(0, 10),
                        new frc.robot.extras.LidarMapComponents.MapPoint(0, 0))
        });

        // Polygon aka hexagon in the middle
        map[1] = new Polygon(new frc.robot.extras.LidarMapComponents.LineSegment[] {
                new frc.robot.extras.LidarMapComponents.LineSegment(
                        new frc.robot.extras.LidarMapComponents.MapPoint(6.000, 5.577),
                        new frc.robot.extras.LidarMapComponents.MapPoint(5.000, 6.155)),
                new frc.robot.extras.LidarMapComponents.LineSegment(
                        new frc.robot.extras.LidarMapComponents.MapPoint(5.000, 6.155),
                        new frc.robot.extras.LidarMapComponents.MapPoint(4.000, 5.577)),
                new frc.robot.extras.LidarMapComponents.LineSegment(
                        new frc.robot.extras.LidarMapComponents.MapPoint(4.000, 5.577),
                        new frc.robot.extras.LidarMapComponents.MapPoint(4.000, 4.423)),
                new frc.robot.extras.LidarMapComponents.LineSegment(
                        new frc.robot.extras.LidarMapComponents.MapPoint(4.000, 4.423),
                        new frc.robot.extras.LidarMapComponents.MapPoint(5.000, 3.845)),
                new frc.robot.extras.LidarMapComponents.LineSegment(
                        new frc.robot.extras.LidarMapComponents.MapPoint(5.000, 3.845),
                        new frc.robot.extras.LidarMapComponents.MapPoint(6.000, 4.423)),
                new frc.robot.extras.LidarMapComponents.LineSegment(
                        new frc.robot.extras.LidarMapComponents.MapPoint(6.000, 4.423),
                        new frc.robot.extras.LidarMapComponents.MapPoint(6.000, 5.577))
        });
        return map;
    }
}