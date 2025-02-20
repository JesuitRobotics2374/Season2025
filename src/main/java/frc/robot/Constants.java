package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.LimelightObject;
import frc.robot.utils.LimelightObject.LLType;
import frc.robot.utils.Setpoint;

public class Constants {

    // Critical Generic Constants

    public static final double MAX_SPEED = 0.2; // kSpeedAt12Volts desired top speed
    public static final double MAX_ANGULAR_RATE = 0.3; // 3/4 of a rotation per second max angular velocity

    public static final double FIELD_X_MIDPOINT = 0; // 8.779; // meters
    public static final double FIELD_Y_MIDPOINT = 0; // 4.026; // meters

    // Elevator

    public static final double ELEVATOR_RATIO = 20;

    // Arm

    public static final double ARM_RATIO = 125;

    public static final double WRIST_PID_P = 0.1; // TODO
    public static final double WRIST_PID_I = 0.0; // TODO
    public static final double WRIST_PID_D = 0.0; // TODO

    public static final double WRIST_MIN_POSITION = 0.0; // TODO: Verify (I think theres a built in encoder)
    public static final double WRIST_MAX_POSITION = 0.235; // TODO: Verify (I think theres a built in encoder)

    public static final double WRIST_INCREMENT = 0.8; // TODO
    public static final double WRIST_MAX_SPEED = 0.2; // TODO
    public static final double WRIST_RATIO = 64;

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
    
    public static final double PATHFINDING_PRE_BUFFER = -1.60116; // meters
    public static final double PATHFINDING_POST_BUFFER = -0.279; // meters
    public static final double PATHFINDING_FRONT_BUFFER = -1.02; // meters
    public static final double PATHFINDING_SHIFT_FACTOR = 0.2351; // meters

    public static final double GENERIC_DISTANCE_THRESHOLD = 0.2;
    public static final double GENERIC_ROTATION_THRESHOLD = 1 * Math.PI / 180;
    public static final double ALIGN_MOVE_SPEED = 0.6;
    public static final double ALIGN_ROTATE_SPEED = 0.01;
    public static final double ALIGN_ROTATIONAL_FEED_FORWARD = 0.8;

    // Other
    
    public static final int SENSOR_PORT = 18;
    public static final String DRIVER_READOUT_TAB_NAME = "Driver Readout";
    

    // Elevator

    public static final int PIGEON_ID = 0;
    public static final double MAX_TIP_ANGLE = 8.0;

    // SETPOINTS

    public static final double RETRACT_ELEVATOR_DOWNSHIFT = 45.0;

    public static final double SETPOINT_ELEVATOR_OFFSET = 0;

    public static final Setpoint SETPOINT_MIN = new Setpoint(5, 22.4, WRIST_MIN_POSITION, "none");
    public static final Setpoint SETPOINT_HP_INTAKE = new Setpoint(20.55, 14.08, WRIST_MAX_POSITION, "none");
    public static final Setpoint SETPOINT_REEF_T1 = new Setpoint(24.83, 1.857, WRIST_MAX_POSITION, "none");
    public static final Setpoint SETPOINT_REEF_T2 = new Setpoint(12.51, 20.37, WRIST_MIN_POSITION, "t2");
    public static final Setpoint SETPOINT_REEF_T3 = new Setpoint(60.76, 18.68, WRIST_MIN_POSITION, "t3");
    public static final Setpoint SETPOINT_REEF_T4 = new Setpoint(123.2, 19.42, WRIST_MIN_POSITION, "t4");
    public static final Setpoint SETPOINT_MAX = new Setpoint(125, 0, WRIST_MIN_POSITION, "none");

} // 19.42
