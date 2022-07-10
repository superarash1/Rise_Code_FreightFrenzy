package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;

public class LookaheadMath {
    // Parameter linePoint1: The starting point of the line segment we are on
// Parameter linePoint2: The ending point of the line segment we are on
// Parameter robotLocation: The coordinate point of the robot location
// Parameter lookaheadDistance: The radius of the circle
    public static ArrayList<Point> lineCircleIntersection(Point linePoint1 , Point linePoint2 , Point robotLocation , double lookaheadDistance){
    /* First we need to account for a line segment on the path that could be straight vertical.
    If this is the case, calculating the slope of this angle would return a divide by
    zero error since the run would be zero in the rise / run formula.*/

    /*We can do this by checking if the difference between the end point x and
    starting point x of the line is very low , indicating the line is vertical.
    If it is, then we can move the line a tiny amount so that the slope wouldn't be 0. */
        if(Math.abs(linePoint1.y - linePoint2.y) < 0.003){
            linePoint1.y = linePoint2.y + 0.003;
        }

        if(Math.abs(linePoint1.x - linePoint2.x) < 0.003){
            linePoint1.x = linePoint2.x + 0.003;
        }

        // Now we can calculate the slope of the line and store it
        // rise / run
        double lineSlope = (linePoint2.y - linePoint1.y) / (linePoint2.x - linePoint1.x);

        // Now that we have the slope of the line, we can calculate the y intercept by isolating b:
    /* y = mx + b
      -mx -mx
      y - mx = b, where (x , y) is any point of the line and m is the slope
    */
        // lets use the linePoint2 as the (x , y) and the lineSlope for m
        double yIntercept = linePoint2.y - lineSlope * linePoint2.x;

        // Now, we can calculate the quadratic A using the equation we found earlier :  m^2 + 1

        double quadraticA = Math.pow(lineSlope , 2) + 1;

        // Calculate the quadratic B using the equation we found earlier :  2m(b - k) - 2h
    /*
        m : slope of the line segment (lineSlope)
        b : y-intercept of the line segment (yIntercept)
        h : x value of the circle center (robotLocation.x)
        k : y value of the circle center (robotLocation.y)
    */

        double quadraticB = 2 * lineSlope * (yIntercept - robotLocation.y) - 2.0 * robotLocation.x;

        // Calculate the quadratic C using the equation we found earlier : (b - k)^2 + h^2 - r^2
    /*
        r : radius of the circle (lookaheadDistance)
    */

        double quadraticC = Math.pow((yIntercept - robotLocation.y) , 2) + Math.pow(robotLocation.x , 2) - Math.pow(lookaheadDistance , 2);

    /*Now that we have the quadraticA , quadraticB , and quadraticC , we can plug these
     values into the quadratic formula to the get the x values of the intersection points*/

        // Let's initialize an array list that we can add the intersection points to
        ArrayList<Point> intersectionPoints = new ArrayList<>();

        try{
        /*Since the quadratic formula has a plus or minus , we can compute two x solutions
        by using one equation with a minus sign and one with a plus sign.
        */

            // Use plus quadratic formula to get the first x solution
            double x1 = (-quadraticB + Math.sqrt(Math.pow(quadraticB , 2) - 4.0 * quadraticA * quadraticC)) / (2.0 * quadraticA);
            // Now that we have the x value of the intersection point on the line, we can plug it into our line equation to get the y value
            double y1 = lineSlope * x1 + yIntercept;

            // Now we have to make sure that the point is in range of the line.
        /* This is because the algorithm assume that the path is a continuous line.
            For example:
                robotLocation : ( 0 , 2)
                linePoint1 : (0 , 0)
                linePoint2 : (0 , 10)
                lookaheadDistance : 5.0
           The lookahead algorithm would calculate to intersections : one at (0 , 7) and one at (0 , -3).
           We don't want the algorithm to recognize (0 , -3) as an intersection since it doesn't
           even exist on the current path we are following. We can do this by setting a minimum and
           maximum value an x value can have which in this case would be both 0 and a minimum and maximum value
           an y value can have which in this case would be 0 and 10.
        */

            // Check which point of the line segment has a lower x value
            double minX = linePoint1.x < linePoint2.x ? linePoint1.x : linePoint2.x;
            // Check which point of the line segment has a higher x value
            double maxX = linePoint1.x > linePoint2.x ? linePoint1.x : linePoint2.x;

            // Check which point of the line segment has a lower y value
            double minY = linePoint1.y < linePoint2.y ? linePoint1.y : linePoint2.y;
            // Check which point of the line segment has a higher y value
            double maxY = linePoint1.y > linePoint2.y ? linePoint1.y : linePoint2.y;

            // If the intersection is in the range of the line segment, then add it as an intersection

            if(x1 > minX && x1 < maxX && y1 > minY && y1 < maxY){
                intersectionPoints.add(new Point(x1 , y1));
            }

            // Now, let's use the minus quadratic formula to find any other existing intersections

            double x2 = (-quadraticB - Math.sqrt(Math.pow(quadraticB , 2) - 4.0 * quadraticA * quadraticC)) / (2.0 * quadraticA);
            double y2 = lineSlope * x2 + yIntercept;

            if(x2 > minX && x2 < maxX && y2 > minY && y2 < maxY){
                intersectionPoints.add(new Point(x2 , y2));
            }

        }catch (Exception e){

        }

        // Return all the intersections

        return intersectionPoints;
    }

    public static Point clipToLine(Point linePoint1 , Point linePoint2 , Point robotLocation){
        // Make sure the start point and end point of the line segment aren't the same
        // If they have same y values or x value s, they can have an undefined or 0 slope
        if(linePoint1.x == linePoint2.x){
            linePoint1.x = linePoint2.x + 0.0001;
        }

        if(linePoint1.y == linePoint2.y){
            linePoint1.y = linePoint2.y + 0.0001;
        }

        // Calculate the slope of the line segment using rise / run
        double m1 = (linePoint2.y - linePoint1.y) / (linePoint2.x - linePoint1.x);
        // Calculate the slope of a perpendicular line by negating the inverse of slope1
        double m2 = -(1 / m1);

        /* Find the y-intercepts, since the end point is a point on the line of the line segment,
        we can use those value as the (x , y) in the equation for b1. Since the robotLocation is on the
        perpendicular line, we can use its (x , y) in the equation for b2.*/
        double b1 = linePoint2.y - m1 * linePoint2.x;
        double b2 = robotLocation.y - m2 * robotLocation.x;

        // Find clipped x using this equation
        double clippedx = (b2 - b1) / (m1 - m2);
        // Find clipped y by pluggin in clipped x as the x value
        double clippedy = (m1 * (clippedx) + b1);

        // Return the point of intersection
        return new Point(clippedx , clippedy);
    }

    // Parameter path : Pass in the path array list that the robot is following
// Parameter robotLocation : Pass in the robot location (x position , y position)
    public static Point clipToPath(ArrayList<Point> path , Point robotLocation){
        // Start off the closest distance really high
        double closestPointDistance = Double.MAX_VALUE;
        // Define the index to a random number so it isn't null
        int currLineIndex = 0;
        // Define a point at a random index so it isn't null. This will store the x and y of the intersection closest to us.
        Point pointOnPath = path.get(0);

        // Iterate through the points of the path
        for(int i = 0 ; i < path.size() - 1; i ++){
            Point startPoint = path.get(i); // The start point of the line segment
            Point endPoint = path.get(i + 1); // The end point of the line segment

            // Intersect the perpendicular line
            Point clippedToLine = clipToLine(startPoint , endPoint , robotLocation);

            // Calulate the distance between the intersection and the robot
            double distanceToPoint = Math.sqrt(Math.pow(clippedToLine.y - robotLocation.y , 2) + Math.pow(clippedToLine.x - robotLocation.x , 2));

            /* If the distance is less than closestPointDistance, set the new closestPointDistance to
             distance and set currIndex to i + 1 and pointOnPath to the clippedLinePoint */
            if(distanceToPoint < closestPointDistance){
                closestPointDistance = distanceToPoint; // Save the closest distance
                pointOnPath = clippedToLine; // Set the x and y of pointOnPath equal to the intersection
                currLineIndex = i + 1; // We are setting it equal to i + 1 because that is the end point (destination) of the line segment
            }
            // Go through loop again
        }

        // Return the point of intersection and index
        return new Point(pointOnPath.x , pointOnPath.y , currLineIndex);
    }

    // Parameter path : Pass in the path array list that the robot is following
// Parameter robotLocation : Pass in the robot location (x position , y position)
    public static Point getLookaheadPoint(ArrayList<Point> path , Point robotLocation , Point currIndexInPath){
    /*Since the clipToPath returns a Point with the values : x (x position of intersection) , y (y position of intersection) , and pointIndex (the current index of the path we are on),
      we can use the pointIndex of the currIndexInPath (calculated using clipToPath) to get the end point of the line segment we are currently on in the path. This gives us control over the loookahead distance of the robot on each line segment since
      we initialize all the path points ourselves. This point will inherit all the values of the point in the path such as lookaheadDistance , movementSpeed etc.
    */
        Point currPointInPath = path.get(currIndexInPath.pointIndex);
        // Start off by setting the lookahead point to something random so it isn't null
        Point lookahead = (path.get(path.size() - 1)); //TODO: I changed this from an error so might be wrong
        // Store the lastPoint as a Point so we can use it later on in the method
        Point lastPoint = path.get(path.size() - 1);

        // Set the closest distance to somethind absurdly high that no intersection is going to be above.
        double closestDistance = 100000000;
        // Iterate through the points in the path
        for(int i = 0; i < path.size() - 1; i++){
            // Get the starting point of the line segment
            Point startPoint = path.get(i);
            // The ending point would jsut be the next point in the path so i + 1
            Point endPoint = path.get(i + 1);

            // Intersect the robot with this line segment.
            /* Pass in the lookahead distance of the current path point we are on.*/
            ArrayList<Point> intersections = lineCircleIntersection(startPoint , endPoint ,
                    robotLocation , currPointInPath.lookaheadDistance);

        /* Iterate through the intersections between the robot and the current line segment.
        It is possible that this list has no intersections in which case the point iteration loop will just move onto
        the next point in the path (i + 1).
        */
            for(Point intersection : intersections){
                // Calculate how far the intersection point is from the end of the path using the distance formula
                double distanceToEnd = Math.sqrt(Math.pow(lastPoint.x - intersection.x , 2) +
                        Math.pow(lastPoint.y - intersection.y , 2));

            /* Check if this iteration is currently the closest one recorded so far
                For example , if the previous line segment's intersection was 10 inches away and the
                current line segment's intersection is 5 inches away, then this intersection is closer than the
                other intersection. Then during the next iteration, the method would check if the intersection is lower than 5.
                This allows us to find the intersection that is closest to the end of the path.
            */
                if(distanceToEnd < closestDistance){
                    // Update the closest distance an intersection point has been recorded to be from the end
                    closestDistance = distanceToEnd;
                    // Set the lookahead point to the x and y of the intersection
                    lookahead.setPoint(intersection);
                }

            }

            // If we are very close to the end of the path, then just set the lookahead to the last point in the path
            if(Math.sqrt(Math.pow(lastPoint.x - robotLocation.x , 2) + Math.pow(lastPoint.y - robotLocation.y , 2)) <= currPointInPath.lookaheadDistance + 5){
                lookahead.setPoint(lastPoint);
            }

            // Inherit faceTowardsAngle and movementSpeed from the line segment we are currently on
            lookahead.faceTowardsAngle = currPointInPath.faceTowardsAngle;
            lookahead.movementSpeed = currPointInPath.movementSpeed;

        }
        // return the calculated lookahead point
        return lookahead;
    }

}
