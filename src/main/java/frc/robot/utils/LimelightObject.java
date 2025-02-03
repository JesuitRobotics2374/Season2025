package frc.robot.utils;

public class LimelightObject {

    public String name;
    public double trust;
    public LLType type;

    public enum LLType {
        kLeft,
        kRight,
        kBack
    }

    public LimelightObject(String name, double trust, LLType type) {
        this.name = name;
        this.trust = 1 / trust;
        this.type = type;
    }
    
}
