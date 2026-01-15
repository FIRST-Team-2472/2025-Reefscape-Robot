package frc.robot.extras.LidarMapComponents;

import frc.robot.extras.LidarMap;

public class LidarSimulator {

    public static double[] getPerfectSimData(MapPoint robotPose, double angle){
        double[] PerfectPointCloud = new double[167];
        for(int i = 0; i < 167; i++){//multiply by .6 because we have one measurement per .6 degrees
            //minus 50 to start at one side of the cone
            PerfectPointCloud[i] = getSmallestRayDistance(LidarMap.getMap()[0],LidarMap.getMap()[1], robotPose, angle + i*.6 -50);
            if(PerfectPointCloud[i] <0.025|| PerfectPointCloud[i] > 0.3){
                PerfectPointCloud[i] = 0;// simulate no return for too close or too far objects
            }
        }
        return PerfectPointCloud;
    }
    public static double[] getMessySimData(MapPoint robotPose, double angle){
        double[] MessyPointCloud = new double[167];
        for(int i = 0; i < 167; i++){//multiply by .6 because we have one measurement per .6 degrees
            //minus 50 to start at one side of the cone
            // random * 6 because angular innacuracy is +-3 degrees
            MessyPointCloud[i] = getSmallestRayDistance(LidarMap.getMap()[0],LidarMap.getMap()[1], robotPose, angle + i*.6 -50 + Math.random()*6 -3);
            if(MessyPointCloud[i] <0.025|| MessyPointCloud[i] > 0.3){
                MessyPointCloud[i] = 0;// simulate no return for too close or too far objects
            }else if(MessyPointCloud[i] <0.1){
                MessyPointCloud[i] += Math.random()*0.003 - 0.0015; //simulate distance noise of up to 3mm
            }else if(MessyPointCloud[i] <0.2){
                MessyPointCloud[i] += MessyPointCloud[i] * Math.random()*0.03 - MessyPointCloud[i]*0.015; //simulate distance noise of up to 3%
            }else if(MessyPointCloud[i] <0.3){
                MessyPointCloud[i] += MessyPointCloud[i] * Math.random()*0.08 - MessyPointCloud[i]*0.04; //simulate distance noise of up to 8%
            }
        }
        return MessyPointCloud;
    }
    // another simulation assuming better data
    public static double[] getLessMessySimData(MapPoint robotPose, double angle){
        double[] lessMessyPointCloud = new double[167];
        for(int i = 0; i < 167; i++){
            lessMessyPointCloud[i] = getSmallestRayDistance(LidarMap.getMap()[0],LidarMap.getMap()[1], robotPose, angle + i*.6 -50 + Math.random()*3 -1.5);
            if(lessMessyPointCloud[i] <0.025|| lessMessyPointCloud[i] > 0.3){
                lessMessyPointCloud[i] = 0;// simulate no return for too close or too far objects
            }else if(lessMessyPointCloud[i] <0.1){
                lessMessyPointCloud[i] += Math.random()*0.002 - 0.001; //simulate distance noise of up to 2mm
            }else if(lessMessyPointCloud[i] <0.2){
                lessMessyPointCloud[i] += lessMessyPointCloud[i] * Math.random()*0.02 - lessMessyPointCloud[i]*0.01; //simulate distance noise of up to 2%
            }else if(lessMessyPointCloud[i] <0.3){
                lessMessyPointCloud[i] += lessMessyPointCloud[i] * Math.random()*0.04 - lessMessyPointCloud[i]*0.02; //simulate distance noise of up to 4%
            }
        }
        return lessMessyPointCloud;
    }



    // --- 2. The Raycasting Logic ---

    /**
     * Casts a ray and finds the shortest distance to any segment in the provided polygons.
     * @param p1 The first polygon.
     * @param p2 The second polygon.
     * @param origin The starting point of the ray.
     * @param angleDeg The angle to cast towards (in degrees).
     * @return The shortest distance, or Double.POSITIVE_INFINITY if nothing is hit.
     */
    public static double getSmallestRayDistance(Polygon p1, Polygon p2, MapPoint origin, double angleDeg) {
        double minDistance = Double.POSITIVE_INFINITY;

        // Convert angle to direction vector
        double angleRad = Math.toRadians(angleDeg);
        double r_dx = Math.cos(angleRad);
        double r_dy = Math.sin(angleRad);

        // Put both polygons in a list to iterate cleanly
        Polygon[] polygonsToCheck = { p1, p2 };

        for (Polygon poly : polygonsToCheck) {
            // Access the array of segments using your specific .get() method
            LineSegment[] segments = poly.get();

            if (segments == null) continue;

            for (LineSegment seg : segments) {
                // Access coordinates using your specific .startPoint.x chain
                double x1 = seg.startPoint.x;
                double y1 = seg.startPoint.y;
                double x2 = seg.endPoint.x;
                double y2 = seg.endPoint.y;

                // --- Standard Ray-Segment Intersection Math ---

                // Segment vector
                double s_dx = x2 - x1;
                double s_dy = y2 - y1;

                // Cross product to check for parallel lines
                // (Ray Direction X Segment Direction)
                double denom = (r_dx * s_dy) - (r_dy * s_dx);

                if (denom == 0) {
                    continue; // Lines are parallel
                }

                // Calculate relative vector from ray origin to segment start
                double rel_x = x1 - origin.x;
                double rel_y = y1 - origin.y;

                // Solve for t (distance along ray)
                double t = (rel_x * s_dy - rel_y * s_dx) / denom;

                // Solve for u (position along segment, must be 0-1)
                double u = (rel_x * r_dy - rel_y * r_dx) / denom;

                // Check intersection validity:
                // t > 0: Intersection is forward of the ray origin
                // 0 <= u <= 1: Intersection lies on the segment
                if (t > 1e-6 && u >= 0 && u <= 1) {
                    if (t < minDistance) {
                        minDistance = t;
                    }
                }
            }
        }

        return minDistance;
    }
}

