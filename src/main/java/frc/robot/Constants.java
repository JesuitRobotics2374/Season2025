package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.LimelightObject;
import frc.robot.utils.LimelightObject.LLType;

public class Constants {

    // Critical Generic Constants

    public static final double MAX_SPEED = 0.4; // kSpeedAt12Volts desired top speed
    public static final double MAX_ANGULAR_RATE = 0.3; // 3/4 of a rotation per second max angular velocity

    // Limelight

    public static final LimelightObject[] LIMELIGHTS_ON_BOARD = {
        new LimelightObject("limelight-left", 1.4, LLType.kLeft),
        // new LimelightObject("limelight-right", 1.1, LLType.kRight),
        // new LimelightObject("limelight-back", 1.4, LLType.kBack)
    };
    public static final Pose2d TEST_PATHFIND_TARGET = new Pose2d(1.199, 7.028, new Rotation2d(128.581 * (Math.PI / 180)));
    
}
