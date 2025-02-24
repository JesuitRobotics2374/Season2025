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
    public static final double MAX_SPEED = 0.2; // kSpeedAt12Volts desired top speed
    public static final double MAX_ANGULAR_RATE = 0.3; // 3/4 of a rotation per second max angular velocity

    public static final double FIELD_X_MIDPOINT = 0; // 8.779; // meters
    public static final double FIELD_Y_MIDPOINT = 0; // 4.026; // meters

    // General Constants
    public static final int SENSOR_PORT = 18;
    public static final String DRIVER_READOUT_TAB_NAME = "Driver Readout";

    // Limelight
    public static final double MIN_CAMERA_DISTANCE = 0; // meters TODO
    public static final LimelightObject[] LIMELIGHTS_ON_BOARD = {
            new LimelightObject("limelight-left", 1.4, LLType.kLeft),
            new LimelightObject("limelight-right", 1.4, LLType.kRight),
            // new LimelightObject("limelight-back", 1.4, LLType.kBack)
    };
    public static final Pose2d TEST_PATHFIND_TARGET = new Pose2d(1.199, 7.028, new Rotation2d(128.581 * (Math.PI / 180)));

    // Arm
    public static final double ARM_RATIO = 125;
    public static final double ARM_HORIZONTAL = 2.666;

    public static final double WRIST_MIN_POSITION = 0.312533;
    public static final double WRIST_MAX_POSITION = 0.570801;

    public static final double WRIST_INCREMENT = 0.8;
    public static final double WRIST_MAX_SPEED = 0.3;
    public static final double WRIST_RATIO = 64;

    // Pathfinding
    public static final double PATHFINDING_MAX_VELOCITY = 3.5;
    public static final double PATHFINDING_MAX_ACCELERATION = 1;
    public static final double PATHFINDING_MAX_ROTATIONAL_VELOCITY = Units.degreesToRadians(540);
    public static final double PATHFINDING_MAX_ROTATIONAL_ACCELERATION = Units.degreesToRadians(720);
    
    public static final double PATHFINDING_PRE_BUFFER = -1.60116; // meters
    public static final double PATHFINDING_POST_BUFFER = -0.279; // meters
    public static final double PATHFINDING_FRONT_BUFFER = -1.02; // meters
    public static final double PATHFINDING_LEFT_SHIFT_FACTOR = -0.2727;
    public static final double PATHFINDING_RIGHT_SHIFT_FACTOR = 0.1037;

    public static final double GENERIC_DISTANCE_THRESHOLD = 0.035; 
    public static final double GENERIC_ROTATION_THRESHOLD = 0.8 * Math.PI / 180;
    public static final double ALIGN_Y_SHIFT = -0.12; //meters limelight
    public static final double ALIGN_MOVE_SPEED = 0.25;
    public static final double ALIGN_ROTATE_SPEED = 0.0006;
    public static final double ALIGN_ROTATIONAL_FEED_FORWARD = 0.25;
    
    // CAN RANGE Movement - Weird Units
    public static final double RIGHT_CANRANGE_OFFSET = -0.04;

    public static final double CAN_RANGE_SPEED = 0.6;
    public static final double CAN_RANGE_FORWARD_DISTANCE = 0.47;
    public static final double CAN_RANGE_BACKWARD_DISTANCE = 1.3;
    
    public static final double SA_RIGHT_BUFFER = -0.0;
    public static final double SA_TARGET_DISTANCE = 0.738;
    public static final double SA_ROTATIONAL_RATE_THRESHOLD = 0.04;

    // Elevator
    public static final double ELEVATOR_RATIO = 20;
    public static final double ELEVATOR_MOVE_AMOUNT = 2;

    public static final int PIGEON_ID = 0;
    public static final double MAX_TIP_ANGLE = 8.0;

    // Setpoints
    public static final double RETRACT_ELEVATOR_DOWNSHIFT = 45.0;
    public static final double SETPOINT_ELEVATOR_OFFSET = 0;

    public static final Setpoint SETPOINT_MIN = new Setpoint(5, 22.4, WRIST_MIN_POSITION, "none");
    public static final Setpoint SETPOINT_HP_INTAKE = new Setpoint(18.55, 14.08, WRIST_MAX_POSITION, "none");
    public static final Setpoint SETPOINT_REEF_T1 = new Setpoint(24.83, 1.857, WRIST_MAX_POSITION, "none");
    public static final Setpoint SETPOINT_REEF_T2 = new Setpoint(12.51, 20.37, WRIST_MIN_POSITION, "t2");
    public static final Setpoint SETPOINT_REEF_T3 = new Setpoint(60.76, 18.68, WRIST_MIN_POSITION, "t3");
    public static final Setpoint SETPOINT_REEF_T4 = new Setpoint(122.2, 20.42, WRIST_MIN_POSITION, "t4");
    public static final Setpoint SETPOINT_MAX = new Setpoint(125, 0, WRIST_MIN_POSITION, "none");
    
}
