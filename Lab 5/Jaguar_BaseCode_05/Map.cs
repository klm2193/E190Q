using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace DrRobot.JaguarControl
{
    public class Map
    {
        public int numMapSegments = 0;
        public double[,,] mapSegmentCorners;
        public double minX, maxX, minY, maxY;
        private double[] slopes;
        private double[] segmentSizes;
        private double[] intercepts;

        private double minWorkspaceX = -10;
        private double maxWorkspaceX =  10;
        private double minWorkspaceY = -10;
        private double maxWorkspaceY =  10;

        public Map()
        {

	        // This is hard coding at its worst. Just edit the file to put in
	        // segments of the environment your robot is working in. This is
	        // used both for visual display and for localization.

	        // ****************** Additional Student Code: Start ************
	
	        // Change hard code here to change map:

	        numMapSegments = 8;
            mapSegmentCorners = new double[numMapSegments, 2, 2];
            slopes = new double[numMapSegments];
            intercepts = new double[numMapSegments];
            segmentSizes = new double[numMapSegments];

            mapSegmentCorners[0, 0, 0] = 3.38 + 5.79 + 3.55 / 2;
	        mapSegmentCorners[0,0,1] = 2.794;
            mapSegmentCorners[0, 1, 0] = -3.38 - 5.79 - 3.55 / 2;
            mapSegmentCorners[0, 1, 1] = 2.794;

	        mapSegmentCorners[1,0,0] = -3.55/2;
	        mapSegmentCorners[1,0,1] = 0.0;
	        mapSegmentCorners[1,1,0] = -3.55/2;
	        mapSegmentCorners[1,1,1] = -2.74;

	        mapSegmentCorners[2,0,0] = 3.55/2;
	        mapSegmentCorners[2,0,1] = 0.0;
	        mapSegmentCorners[2,1,0] = 3.55/2;
	        mapSegmentCorners[2,1,1] = -2.74;

            mapSegmentCorners[3, 0, 0] = 3.55/2;
            mapSegmentCorners[3, 0, 1] = 0.0;
            mapSegmentCorners[3, 1, 0] = 3.55 / 2 + 5.79;
            mapSegmentCorners[3, 1, 1] = 0.0;

            mapSegmentCorners[4, 0, 0] = -3.55/2;
            mapSegmentCorners[4, 0, 1] = 0.0;
            mapSegmentCorners[4, 1, 0] = -3.55/2 - 5.79;
            mapSegmentCorners[4, 1, 1] = 0.0;

            mapSegmentCorners[5, 0, 0] = -3.55/2;
            mapSegmentCorners[5, 0, 1] = -2.74;
            mapSegmentCorners[5, 1, 0] = -3.55/2-3.05;
            mapSegmentCorners[5, 1, 1] = -2.74;

            mapSegmentCorners[6, 0, 0] = 3.55 / 2;
            mapSegmentCorners[6, 0, 1] = -2.74;
            mapSegmentCorners[6, 1, 0] = 3.55 / 2 + 3.05;
            mapSegmentCorners[6, 1, 1] = -2.74;

            mapSegmentCorners[7, 0, 0] = 5.03 / 2;
            mapSegmentCorners[7, 0, 1] = -2.74 - 2.31;
            mapSegmentCorners[7, 1, 0] = -5.03/2;
            mapSegmentCorners[7, 1, 1] = -2.74 - 2.31;
            // ****************** Additional Student Code: End   ************


	        // Set map parameters
	        // These will be useful in your future coding.
	        minX = 9999; minY = 9999; maxX=-9999; maxY=-9999;
	        for (int i=0; i< numMapSegments; i++){
		
		        // Set extreme values
                minX = Math.Min(minX, Math.Min(mapSegmentCorners[i,0,0], mapSegmentCorners[i,1,0]));
                minY = Math.Min(minY, Math.Min(mapSegmentCorners[i,0,1], mapSegmentCorners[i,1,1]));
                maxX = Math.Max(maxX, Math.Max(mapSegmentCorners[i,0,0], mapSegmentCorners[i,1,0]));
                maxY = Math.Max(maxY, Math.Max(mapSegmentCorners[i,0,1], mapSegmentCorners[i,1,1]));
		
		        // Set wall segments to be horizontal
		        slopes[i] = (mapSegmentCorners[i,0,1]-mapSegmentCorners[i,1,1])/(0.001+mapSegmentCorners[i,0,0]-mapSegmentCorners[i,1,0]);
		        intercepts[i] = mapSegmentCorners[i,0,1] - slopes[i]*mapSegmentCorners[i,0,0];

		        // Set wall segment lengths
		        segmentSizes[i] = Math.Sqrt(Math.Pow(mapSegmentCorners[i,0,0]-mapSegmentCorners[i,1,0],2)+Math.Pow(mapSegmentCorners[i,0,1]-mapSegmentCorners[i,1,1],2));
	        }
        }


        // This function is used in your particle filter localization lab. Find 
        // the range measurement to a segment given the ROBOT POSITION (x, y) and 
        // SENSOR ORIENTATION (t)
        double GetWallDistance(double x, double y, double t, int segment)
        {
            // ****************** Additional Student Code: Start ************

            double maxDist = 9999;

            //not get infinity from the tangent function
            if (Math.Abs(t) == Math.PI / 2)
            {
                t = (Math.PI / 2) + 0.001;
            }

            double m1 = Math.Tan(t); //slope of laser range measurement
            double m2 = Math.Max(slopes[segment], 0.001); // slope of the segment; don't let it be 0
            double d; //distance from robot to intersection point

            //the lines are parallel, so there is no intersection
            if (m1 == m2)
            {
                d = maxDist;
            }

            //the lines have an intersection point
            else
            {
                double xSegment1 = mapSegmentCorners[segment, 0, 0]; // x value of end 1 on segment
                double ySegment1 = mapSegmentCorners[segment, 0, 1]; // y value of end 1 on segment
                double xSegment2 = mapSegmentCorners[segment, 1, 0]; // x value of end 2 on segment
                double ySegment2 = mapSegmentCorners[segment, 1, 1]; // y value of end 2 on segment

                // calculate x and y intersection points
                double intersectionX = (m1 * x - y - m2 * xSegment1 + ySegment1) / (m1 - m2);
                double intersectionY = ((y / m1) - x - (ySegment1 / m2) + xSegment1) / ((1 / m1) - (1 / m2));

                double segmentLength = Math.Sqrt(Math.Pow((xSegment1 - xSegment2), 2) + Math.Pow((ySegment1 - ySegment2), 2));

                // Calculating the min and max x and y coordinates for a segment
                /*
                minX = Math.Min(minX, Math.Min(mapSegmentCorners[segment,0,0], mapSegmentCorners[segment,1,0]));
                minY = Math.Min(minY, Math.Min(mapSegmentCorners[segment,0,1], mapSegmentCorners[segment,1,1]));
                maxX = Math.Max(maxX, Math.Max(mapSegmentCorners[segment,0,0], mapSegmentCorners[segment,1,0]));
                maxY = Math.Max(maxY, Math.Max(mapSegmentCorners[segment,0,1], mapSegmentCorners[segment,1,1]));
                 */

                // Angle of the Intersection Point
                double AngleIntersectionPoint = Math.Atan2(intersectionY - y, intersectionX - x);

                double deltaL = Math.Sqrt(Math.Pow((intersectionX - xSegment1), 2) + Math.Pow((intersectionY - ySegment1), 2));
                double deltaR = Math.Sqrt(Math.Pow((intersectionX - xSegment2), 2) + Math.Pow((intersectionY - ySegment2), 2));

                //Making sure that intersection point is within the bounds of the segment and the target is not behind the robot
                if ((Math.Max(deltaL, deltaR) < segmentLength) && (Math.Abs(AngleIntersectionPoint - t) < 0.1))
                {
                    d = Math.Sqrt(Math.Pow((x - intersectionX), 2) + Math.Pow((y - intersectionY), 2));
                }

                else
                {
                    d = maxDist;
                }

                double dummyD = d;
                dummyD += dummyD;

            }
            // ****************** Additional Student Code: End   ************

            return d;
        }


        // This function is used in particle filter localization to find the
        // range to the closest wall segment, for a robot located
        // at position x, y with sensor with orientation t.

        public double GetClosestWallDistance(double x, double y, double t)
        {
            double minDist = 9999;// 6.000;

            // ****************** Additional Student Code: Start ************

            // Put code here that loops through segments, calling the
            // function GetWallDistance.

            for (int segment = 0; segment < numMapSegments; segment++)
            {
                double wallDistance = GetWallDistance(x, y, t, segment);
                minDist = Math.Min(minDist, wallDistance);
            }

            // ****************** Additional Student Code: End   ************

            return minDist;
        }

        public double GetClosestWallDistanceSimRobot(double x, double y, double t)
        {

            double minDist = 6.000;

            // ****************** Additional Student Code: Start ************

            // Put code here that loops through segments, calling the
            // function GetWallDistance.

            for (int segment = 0; segment < numMapSegments; segment++)
            {
                double wallDistance = GetWallDistance(x, y, t, segment);
                minDist = Math.Min(minDist, wallDistance);
            }

            // ****************** Additional Student Code: End   ************

            return minDist;
        }

        // This function is called from the motion planner. It is
        // used to check for collisions between an edge between
        // nodes n1 and n2, and a wall segment.
        // The function uses an iterative approach by moving along
        // the edge a "safe" distance, defined to be the shortest distance 
        // to the wall segment, until the end of the edge is reached or 
        // a collision occurs.

        public bool CollisionFound(Navigation.Node n1, Navigation.Node n2, double tol)
        {


            // Check that within boundaries
            if (n2.x > maxWorkspaceX || n2.x < minWorkspaceX || n2.y > maxWorkspaceY || n2.y < minWorkspaceY)
                return true;


            // Check for collision with walls
            double theta = Math.Atan2(n2.y - n1.y, n2.x - n1.x);
            double edgeSize = Math.Sqrt(Math.Pow(n2.y - n1.y, 2) + Math.Pow(n2.x - n1.x, 2));
            double sinTheta = Math.Sin(theta);
            double cosTheta = Math.Cos(theta);

            // Loop through segments
            for (int segment = 0; segment < numMapSegments; segment++)
            {

                double distTravelledOnEdge = 0;
                double ex = n1.x, ey = n1.y;
                double distToSegment;
                while (distTravelledOnEdge - tol < edgeSize)
                {
                    distToSegment = GetWallDistance(ex, ey, segment, tol, n2.x, n2.y);
                    if (distToSegment - tol < 0.05)
                        return true;
                    ex += cosTheta * distToSegment;
                    ey += sinTheta * distToSegment;
                    distTravelledOnEdge += distToSegment;
                }

            }
            return false;
        }


        // This function will calculate the length of the perpendicular 
        // connecting point x,y to the wall segment. If the perpendicular
        // does not hit the segment, a large number is returned.

        double GetWallDistance(double x, double y, int segment, double tol, double n2x, double n2y){
            // Set wall vars
            double X1 = mapSegmentCorners[segment, 0, 0];
            double Y1 = mapSegmentCorners[segment, 0, 1];
            double X2 = mapSegmentCorners[segment, 1, 0];
            double Y2 = mapSegmentCorners[segment, 1, 1];
            double dist = 9999;

            // Put code here to calculated dist.
            // Calculate slope and intercept
            double angleSegmentPerpendicular = Math.PI / 2 + Math.Atan((Y2 - Y1) / (0.000001 + X2 - X1));
            double m = Math.Tan(angleSegmentPerpendicular);
            double b = y - m * x;

            // Get line intersection
            double x_intersect = (b - intercepts[segment]) / (slopes[segment] - m);
            double y_intersect = m * x_intersect + b;

            // Check for horiz/vert slopes
            if (Math.Abs(Y2 - Y1) < 0.001)
                y_intersect = Y1;
            if (Math.Abs(X2 - X1) < 0.001)
                x_intersect = X1;


            // Check to see if intersection LIES within segment
            double dist_intersect_corner1 = Math.Sqrt(Math.Pow(x_intersect - X1, 2) + Math.Pow(y_intersect - Y1, 2));
            double dist_intersect_corner2 = Math.Sqrt(Math.Pow(x_intersect - X2, 2) + Math.Pow(y_intersect - Y2, 2));
            if (dist_intersect_corner1 <= (segmentSizes[segment] + tol) && dist_intersect_corner2 <= (segmentSizes[segment] + tol))
            {
                dist = Math.Sqrt(Math.Pow(x - x_intersect, 2) + Math.Pow(y - y_intersect, 2));
            }

            // Check for distance to corners (for case where no intersection with segment
            double dist_point_corner1 = Math.Sqrt(Math.Pow(x - X1, 2) + Math.Pow(y - Y1, 2));
            double dist_point_corner2 = Math.Sqrt(Math.Pow(x - X2, 2) + Math.Pow(y - Y2, 2));
            dist = Math.Min(dist, dist_point_corner1);
            dist = Math.Min(dist, dist_point_corner2);

            return dist;
        }


    }
}
