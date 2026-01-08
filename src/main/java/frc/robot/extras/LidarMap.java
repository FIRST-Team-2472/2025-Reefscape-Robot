package frc.robot.extras;

import frc.robot.extras.LidarMapComponents.Polygon;

public class LidarMap {
    public Polygon[] map = new Polygon[3];

    public LidarMap() {//all numbers are magic so far
        // Define polygons and add them to the map array
        // Polygon 1 aka the field walls
        map[0] = new Polygon(new frc.robot.extras.LidarMapComponents.LineSegment[] {
                new frc.robot.extras.LidarMapComponents.LineSegment(
                        new frc.robot.extras.LidarMapComponents.MapPoint(0, 0),
                        new frc.robot.extras.LidarMapComponents.MapPoint(100, 0)),
                new frc.robot.extras.LidarMapComponents.LineSegment(
                        new frc.robot.extras.LidarMapComponents.MapPoint(100, 0),
                        new frc.robot.extras.LidarMapComponents.MapPoint(100, 100)),
                new frc.robot.extras.LidarMapComponents.LineSegment(
                        new frc.robot.extras.LidarMapComponents.MapPoint(100, 100),
                        new frc.robot.extras.LidarMapComponents.MapPoint(0, 100)),
                new frc.robot.extras.LidarMapComponents.LineSegment(
                        new frc.robot.extras.LidarMapComponents.MapPoint(0, 100),
                        new frc.robot.extras.LidarMapComponents.MapPoint(0, 0))
        });

        // Polygon 2
        map[1] = new Polygon(new frc.robot.extras.LidarMapComponents.LineSegment[] {
                new frc.robot.extras.LidarMapComponents.LineSegment(
                        new frc.robot.extras.LidarMapComponents.MapPoint(150, 150),
                        new frc.robot.extras.LidarMapComponents.MapPoint(250, 150)),
                new frc.robot.extras.LidarMapComponents.LineSegment(
                        new frc.robot.extras.LidarMapComponents.MapPoint(250, 150),
                        new frc.robot.extras.LidarMapComponents.MapPoint(250, 250)),
                new frc.robot.extras.LidarMapComponents.LineSegment(
                        new frc.robot.extras.LidarMapComponents.MapPoint(250, 250),
                        new frc.robot.extras.LidarMapComponents.MapPoint(150, 250)),
                new frc.robot.extras.LidarMapComponents.LineSegment(
                        new frc.robot.extras.LidarMapComponents.MapPoint(150, 250),
                        new frc.robot.extras.LidarMapComponents.MapPoint(150, 150))
        });

        // Polygon 3
        map[2] = new Polygon(new frc.robot.extras.LidarMapComponents.LineSegment[] {
                new frc.robot.extras.LidarMapComponents.LineSegment(
                        new frc.robot.extras.LidarMapComponents.MapPoint(300, 300),
                        new frc.robot.extras.LidarMapComponents.MapPoint(400, 300)),
                new frc.robot.extras.LidarMapComponents.LineSegment(
                        new frc.robot.extras.LidarMapComponents.MapPoint(400, 300),
                        new frc.robot.extras.LidarMapComponents.MapPoint(400, 400)),
                new frc.robot.extras.LidarMapComponents.LineSegment(
                        new frc.robot.extras.LidarMapComponents.MapPoint(400, 400),
                        new frc.robot.extras.LidarMapComponents.MapPoint(300, 400)),
                new frc.robot.extras.LidarMapComponents.LineSegment(
                        new frc.robot.extras.LidarMapComponents.MapPoint(300, 400),
                        new frc.robot.extras.LidarMapComponents.MapPoint(300, 300))
        });
    }
}