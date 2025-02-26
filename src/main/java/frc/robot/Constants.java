package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.seafinder.utils.Setpoint;
import frc.robot.utils.LimelightObject;
import frc.robot.utils.LimelightObject.LLType;

public class Constants {
    // Critical Generic Constants
    public static final double MAX_SPEED_LOW_ELEVATOR = 0.86; // movement rate in meters 
    public static final double MAX_SPEED_HIGH_ELEVATOR = 0.12; // movement rate in meters (when elevator is high, to avoid tipping)

    public static final double MAX_ANGULAR_RATE = 0.45; // angular rate in rotations per second

    public static final double FIELD_X_MIDPOINT = 0; // 8.779; // meters (currently registers correctly at bottom left)
    public static final double FIELD_Y_MIDPOINT = 0; // 4.026; // meters

    // General Constants
    public static final int PIGEON_ID = 0;

    // Limelight
    public static final LimelightObject[] LIMELIGHTS_ON_BOARD = {
            new LimelightObject("limelight-left", 1.4, LLType.kLeft),
            new LimelightObject("limelight-right", 1.4, LLType.kRight),
            // new LimelightObject("limelight-back", 1.4, LLType.kBack)
    };

    // Arm
    public static final double ARM_RATIO = 125;
    public static final double ARM_HORIZONTAL = 2.666; // position of the arm MOTOR when horizontal (before ratio)

    public static final double WRIST_RATIO = 64;
    public static final double WRIST_MIN_POSITION = 0.337158; // position of the wrist AFTER ratio, horizontal
    public static final double WRIST_MAX_POSITION = 0.571045; // position of the wrist AFTER ratio, vertical

    public static final double WRIST_INCREMENT = 0.8; // increment of wrist position when moved manually

    // Pathfinding
    public static final double PATHFINDING_MAX_VELOCITY = 2.6; // meters per second
    public static final double PATHFINDING_MAX_ACCELERATION = 2.2; // meters per second squared
    public static final double PATHFINDING_MAX_ROTATIONAL_VELOCITY = Units.degreesToRadians(540); // degrees per second
    public static final double PATHFINDING_MAX_ROTATIONAL_ACCELERATION = Units.degreesToRadians(720); // degrees per second squared
    
    public static final double PATHFINDING_PRE_BUFFER = -1.60116; // meters
    public static final double PATHFINDING_POST_BUFFER = -0.279; // meters
    public static final double PATHFINDING_FRONT_BUFFER = -1.02; // meters
    public static final double PATHFINDING_LEFT_SHIFT_FACTOR = -0.2867; // meters
    public static final double PATHFINDING_RIGHT_SHIFT_FACTOR = 0.13; // meters

    public static final double GENERIC_DISTANCE_THRESHOLD = 0.035; // meters 
    public static final double GENERIC_ROTATION_THRESHOLD = 0.8 * Math.PI / 180; // radians
    public static final double ALIGN_Y_SHIFT = -0.12; // meters
    public static final double ALIGN_MOVE_SPEED = 0.35; // meters per second
    public static final double ALIGN_ROTATE_SPEED = 3; // degrees per second
    public static final double ALIGN_ROTATIONAL_FEED_FORWARD = 0.2; // radians per second

    public static final double STATIC_BACK_TIME = 0.6; // seconds
    
    // Front CAN Ranges
    public static final double RIGHT_CAN_RANGE_OFFSET = -0.04; // meters

    public static final double CAN_RANGE_FORWARD_DISTANCE = 0.47; // meters
    public static final double CAN_RANGE_BACKWARD_DISTANCE = 1.3; // meters
    
    public static final double STATION_TARGET_DISTANCE = 0.69; // meters
    public static final double STATION_ROTATIONAL_RATE_THRESHOLD = 0.04; // radians

    // Elevator
    public static final double ELEVATOR_RATIO = 20;
    public static final double ELEVATOR_MOVE_AMOUNT = 2; // increment of elevator position when moved manually

    public static final double MAX_TIP_ANGLE = 8.0; // degrees

    public static final double ELEVATOR_MAX_HEIGHT = 128.2; // rotations of elevator motor BEFORE ratio

    public static final double RETRACT_ELEVATOR_DOWNSHIFT = 45.0;
    public static final double SETPOINT_ELEVATOR_OFFSET = 0;

    // Arm
    public static final double ARM_MOVE_AMOUNT = 2;

    // Setpoints

    // Setpoints describe specific robot state positions
    public static final Setpoint SETPOINT_MIN = new Setpoint(5, 22.4, WRIST_MIN_POSITION, "none");
    public static final Setpoint SETPOINT_HP_INTAKE = new Setpoint(18.55, 15.2, WRIST_MAX_POSITION, "none");
    public static final Setpoint SETPOINT_PROCESSOR = new Setpoint(25, 2, WRIST_MIN_POSITION, "none");
    public static final Setpoint SETPOINT_BARGE = new Setpoint(128.1, 22.26, WRIST_MAX_POSITION, "none");
    public static final Setpoint SETPOINT_REEF_T1 = new Setpoint(24.83, 1.857, WRIST_MAX_POSITION, "none");
    public static final Setpoint SETPOINT_REEF_T2 = new Setpoint(12.51, 20.37, WRIST_MIN_POSITION, "t2");
    public static final Setpoint SETPOINT_REEF_T3 = new Setpoint(60.76, 18.68, WRIST_MIN_POSITION, "t3");
    public static final Setpoint SETPOINT_REEF_T4 = new Setpoint(122.2, 20.42, WRIST_MIN_POSITION, "t4");
    public static final Setpoint SETPOINT_ALGAE_T2 = new Setpoint(74.5, -1.81, WRIST_MAX_POSITION, "c2");
    public static final Setpoint SETPOINT_ALGAE_T3 = new Setpoint(110.76, -2.68, WRIST_MAX_POSITION, "c3");
    public static final Setpoint SETPOINT_MAX = new Setpoint(125, 0, WRIST_MIN_POSITION, "none");
    
}
