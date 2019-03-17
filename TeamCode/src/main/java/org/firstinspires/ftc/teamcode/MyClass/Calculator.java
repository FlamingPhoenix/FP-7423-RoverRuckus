package org.firstinspires.ftc.teamcode.MyClass;

public class Calculator {
    public static float getMineralAngle(float distanceFromLander){
        float adjacentWidth = 36f * (float)Math.sqrt(2f) - 22.8f/2f - distanceFromLander - 9f;
        double rAngle = Math.atan2(14.5, adjacentWidth);

        return (float) Math.toDegrees(rAngle);
    }
}
