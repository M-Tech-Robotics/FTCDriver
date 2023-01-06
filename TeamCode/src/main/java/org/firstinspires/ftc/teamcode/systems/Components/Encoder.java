package org.firstinspires.ftc.teamcode.systems.Components;

import com.acmerobotics.roadrunner.geometry.Vector2d;

public class Encoder {
    public double ticksToInches;
    public int lastPos;
    public int currentPos;
    public double scaleFactor;
    public double x;
    public double y;

    public Encoder (Vector2d point, double scaleFactor){
        double ticksPerRotation = 8192;
        double wheelRadius = 1.37795;
        ticksToInches = (wheelRadius * Math.PI * 2.0)/ticksPerRotation; //72.0/133000.0 this origional calculation was off by 2%
        x = point.getX();
        y = point.getY();
        currentPos = 0;
        lastPos = currentPos;
        this.scaleFactor = scaleFactor;
    }

    public void update(int Pos){
        lastPos = currentPos;
        currentPos = Pos;
    }
    
    public double getDelta(){
        return (double)(currentPos-lastPos)*ticksToInches*scaleFactor;
    }
    public double getCurrentDist(){
        return (double)(currentPos)*ticksToInches*scaleFactor;
    }
}
