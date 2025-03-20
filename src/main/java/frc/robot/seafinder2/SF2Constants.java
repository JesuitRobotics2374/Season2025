package frc.robot.seafinder2;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.seafinder2.utils.Target.Setpoint;
import frc.robot.utils.LimelightObject;

public class SF2Constants {

    public static final LimelightObject[] LIMELIGHTS_ON_BOARD = Constants.LIMELIGHTS_ON_BOARD; // Which Limelights should SF2 use?

    public static final double SEAFINDER2_MAX_VELOCITY = 4.5;
    public static final double SEAFINDER2_MAX_ACCELERATION = 5;
    public static final double SEAFINDER2_MAX_ROTATIONAL_VELOCITY = Units.degreesToRadians(720);
    public static final double SEAFINDER2_MAX_ROTATIONAL_ACCELERATION = Units.degreesToRadians(940);

    // .55 is flush against reeff
    //public static final double SEAFINDER2_REEF_FRONT_PADDING = -0.55 ;  //0.55 sems to work for L2,L3
    public static final double SEAFINDER2_REEF_FRONT_PADDING = -0.55;  //0.75 seems correct for L4
    public static final double SEAFINDER2_ASTAR_PADDING = -0.9;
  //  public static final double SEAFINDER2_REEF_FRONT_PADDING = -1.0;  //safety
    public static final double SEAFINDER2_REEF_LEFT_BRANCH_OFFSET = -0.1363;
    public static final double SEAFINDER2_REEF_RIGHT_BRANCH_OFFSET = 0.1464;

    public static final double WRIST_MIN_POSITION = 0.0;
    public static final double WRIST_MAX_POSITION = 0.25;

    public static final Setpoint SETPOINT_MIN = new Setpoint(5, 22.4, WRIST_MIN_POSITION);
    public static final Setpoint SETPOINT_HP_INTAKE = new Setpoint(0.00, 14.08, WRIST_MAX_POSITION);
    public static final Setpoint SETPOINT_PROCESSOR = new Setpoint(33.27, 2, WRIST_MAX_POSITION);
    public static final Setpoint SETPOINT_BARGE = new Setpoint(128.1, 22.26, WRIST_MAX_POSITION);
    public static final Setpoint SETPOINT_REEF_T1 = new Setpoint(14, 1.857, WRIST_MAX_POSITION);
    public static final Setpoint SETPOINT_REEF_T2 = new Setpoint(12.51, 20.37, WRIST_MIN_POSITION);
    public static final Setpoint SETPOINT_REEF_T3 = new Setpoint(60.76 - 38, 18.68, WRIST_MIN_POSITION);
    public static final Setpoint SETPOINT_REEF_T4 = new Setpoint(95.78, 19.17, WRIST_MIN_POSITION); //around 110 //12.25
    public static final Setpoint SETPOINT_ALGAE_T2 = new Setpoint(74.5, -1.81, WRIST_MAX_POSITION);
    public static final Setpoint SETPOINT_ALGAE_T3 = new Setpoint(110.76, -2.68, WRIST_MAX_POSITION);
    public static final Setpoint SETPOINT_MAX = new Setpoint(125, 0, WRIST_MIN_POSITION);

    public static final double CAN_RANGE_FORWARD_DISTANCE = 0.53;
    
}
