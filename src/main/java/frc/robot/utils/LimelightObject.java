package frc.robot.utils;

public class LimelightObject {

    public String name;
    public double trust;
    public LLType type;

    private boolean available = true;

    public enum LLType {
        kLeft,
        kRight,
        kBack
    }

    public LimelightObject(String name, double trust, LLType type) {
        this.name = name;
        this.trust = 1 / trust;
        this.type = type;
        try {
            LimelightHelpers.getBotPose(name);
        } catch (Exception e) {
            available = false;
        }
    }

    public boolean isAvailable() {
        return available;
    }
    
}
