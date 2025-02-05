package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.LimelightObject;
import frc.robot.utils.LimelightObject.LLType;

public class Constants {

    // Critical Generic Constants

    public static final double MAX_SPEED = 0.5; // kSpeedAt12Volts desired top speed
    public static final double MAX_ANGULAR_RATE = 0.3; // 3/4 of a rotation per second max angular velocity

    public static final double FIELD_X_MIDPOINT = 8.779; // meters
    public static final double FIELD_Y_MIDPOINT = 4.026; // meters

    // Limelight

    public static final LimelightObject[] LIMELIGHTS_ON_BOARD = {
        new LimelightObject("limelight-left", 1.4, LLType.kLeft),
        // new LimelightObject("limelight-right", 1.1, LLType.kRight),
        // new LimelightObject("limelight-back", 1.4, LLType.kBack)
    };
    public static final Pose2d TEST_PATHFIND_TARGET = new Pose2d(1.199, 7.028, new Rotation2d(128.581 * (Math.PI / 180)));

    // Pathfinding

    public static final double PATHFINDING_MAX_VELOCITY = 3.5;
    public static final double PATHFINDING_MAX_ACCELERATION = 1;
    public static final double PATHFINDING_MAX_ROTATIONAL_VELOCITY = Units.degreesToRadians(540);
    public static final double PATHFINDING_MAX_ROTATIONAL_ACCELERATION = Units.degreesToRadians(720);
    
    public static final double PATHFINDING_PRE_BUFFER = -2.14116; // meters
    public static final double PATHFINDING_FRONT_BUFFER = -0.42; // meters
    public static final double PATHFINDING_SHIFT_FACTOR = 0.1951; // meters
    
    public static final int SENSOR_PORT = 18;
    public static final String DRIVER_READOUT_TAB_NAME = "Driver Readout";

    // Elevator

    public static final int PIGEON_ID = 29;
    public static final double MAX_TIP_ANGLE = 8.0;

}
