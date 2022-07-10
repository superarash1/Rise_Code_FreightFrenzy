package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;

public class Point {
    public double x;
    public double y;
    public double movementSpeed;
    public double faceTowardsAngle;
    public double lookaheadDistance;
    public int pointIndex;

    public Point(double x , double y){
        this.x = x;
        this.y = y;
    }

    public Point(double x , double y , double movementSpeed , double faceTowardsAngle , double lookaheadDistance){
        this.x = x;
        this.y = y;
        this.movementSpeed = movementSpeed;
        this.faceTowardsAngle = faceTowardsAngle;
        this.lookaheadDistance = lookaheadDistance;
    }

    public Point(double x , double y , int pointIndex){
        this.x = x;
        this.y = y;
        this.pointIndex = pointIndex;
    }

    public void setPoint(Point point){
        this.x = point.x;
        this.y = point.y;
    }

    // Initializing the path array list
    ArrayList<Point> path = new ArrayList<>();


}
