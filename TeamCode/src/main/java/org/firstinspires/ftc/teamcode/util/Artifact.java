package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class Artifact {
    private double red;
    private double green;
    private double blue;
    private double displacement;
    public static double currentDisplacement = 0;
    public Artifact(NormalizedRGBA color){
        red = color.red;
        green = color.green;
        blue = color.blue;
        displacement = currentDisplacement;

    }
    public String getColor(){
        if(red>green && blue>green)
            return "Purple";
        if(green>red && blue>red)
            return "Green";
        return null;
    }
    public double getDisplacement(){
        return displacement;
    }

}
