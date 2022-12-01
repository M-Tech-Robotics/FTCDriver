package org.firstinspires.ftc.teamcode.systems.Utils.Objects;

import com.acmerobotics.roadrunner.geometry.Vector2d;

public class Encoder {
    public double ticksToInches;
    public int lastVal;
    public int currentVal;
    public double scaleFactor;
    public double X;
    public double Y;
    
    
    public Encoder(Vector2d point, double scaleFactor){
        double ticksPerRotation = 0; // Add Data for Odometry
        double wheelRadius = 0;

        ticksToInches = (wheelRadius * Math.PI * 2.0)/ticksPerRotation;
        X = point.getX();
        Y = point.getY();
        currentVal = 0;
        lastVal = currentVal;

        this.scaleFactor = scaleFactor;
    }
    
    
    public void update(int currentPos){
        lastVal = currentVal;
        currentVal = currentPos;
    }
    
    
    public double getDelta(){
        return (double)(currentVal - lastVal) * ticksToInches * scaleFactor;
    }
    
    public double getCurrentDist(){
        return (double)(currentVal) * ticksToInches * scaleFactor;
    }
}
