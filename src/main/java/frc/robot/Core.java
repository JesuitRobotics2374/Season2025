// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.lang.management.OperatingSystemMXBean;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.seafinder2.PathfinderSubsystem;
import frc.robot.seafinder2.interfaces.PanelSubsystem;
import frc.robot.seafinder2.utils.Target;
import frc.robot.seafinder2.utils.Target.Landmark;
import frc.robot.seafinder2.utils.Target.Location;
import frc.robot.seafinder2.utils.Target.Side;
import frc.robot.seafinder2.utils.Target.TagRelativePose;
// import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.seafinder2.SF2Constants;
import frc.robot.seafinder2.commands.CanRangeStation;
import frc.robot.seafinder2.commands.ExactAlign;
import frc.robot.seafinder2.commands.ScoreCommand;
import frc.robot.seafinder2.commands.TestCommand;
import frc.robot.seafinder2.commands.limbControl.EjectCommand;
import frc.robot.seafinder2.commands.limbControl.ElevatorCommand;

public class Core {

    public double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * Constants.MAX_SPEED;

    public double MaxSpeedTurbo = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * Constants.MAX_SPEED_TURBO;

    public boolean isTurbo = false;

    public double currentElevatorPosition = 0;

    public double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond) * Constants.MAX_ANGULAR_RATE;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.02) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    // private final SwerveRequest.SwerveDriveBrake brake = new
    // SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new
    // SwerveRequest.PointWheelsAt();

    // private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    public final ManipulatorSubsystem manipulatorSubsystem = new ManipulatorSubsystem();
    // public final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

    public final PathfinderSubsystem pathfinderSubsystem = new PathfinderSubsystem(this);

    // public final PanelSubsystem panelSubsystem = new PanelSubsystem(pathfinderSubsystem);

    public SequentialCommandGroup autoCommandGroup;

    Pose3d llp;
    Pose3d llp2;
    // private final SendableChooser<Command> autoChooser;

    private Command pathfindingCommand;

    private Target target1;
    private Target target2;


    public AprilTagFieldLayout atf;

    private TagRelativePose assistiveEAPose;

    public Core() {

        target1 = new Target(this);
        target1.setLocation(new Target.Location(Landmark.REEF_BACK, Side.RIGHT));
        target1.setHeight(Target.Height.BRANCH_L4); // This is a structural requirement, but we don't use it here.

        target2 = new Target(this);
        target2.setLocation(new Target.Location(Landmark.REEF_FRONT_RIGHT, Side.RIGHT));
        target2.setHeight(Target.Height.BRANCH_L4); // This is a structural requirement, but we don't use it here.

        registerAutoCommands();
        // autoChooser = AutoBuilder.buildAutoChooser();
        configureBindings();
        configureShuffleBoard();

        atf = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

        // drivetrain.setRobotPose(new Pose2d(7.5, 1.5, new Rotation2d(180 * (Math.PI /
        // 180))));

        // DEBUG

        ShuffleboardTab tab = Shuffleboard.getTab("Test");

        if (llp != null) {

            // LL Outs
            tab.addDouble("EE LL X", () -> {
                return llp.getX();
            });
            tab.addDouble("EE LL Y", () -> {
                return llp.getY();
            });
            tab.addDouble("EE LL Yaw", () -> {
                return llp.getRotation().getZ();
            });

        }
        autoCommandGroup = pathfinderSubsystem.autoSequence;
    }

    // A setpoint is a "macro" state. Find its definition in utils folder.
    // public void moveToSetpoint(Setpoint setpoint) {
    //     queuedRetractAction = setpoint.getRetractAction(); // Store what we just did for when we retract
    //     elevatorSubsystem.elevatorGoToDouble(setpoint.getElevator());
    //     armSubsystem.armGoTo(setpoint.getArm());
    //     armSubsystem.wristGoTo(setpoint.getWrist());
    // }
    
    public void moveElevatorOnly(double val) {
        elevatorSubsystem.elevatorGoToDouble(manipulatorSubsystem, val);
    }


    public void registerAutoCommands() {
        // NamedCommands.registerCommand("OuttakeCommand", new
        // Outtake(outtakeSubsystem));
        // NamedCommands.registerCommand("Test Pathfind", new PathfindBasic(drivetrain,
        // Constants.TEST_PATHFIND_TARGET));

        // PathfindingCommand.warmupCommand().schedule();
    }

    public void configureShuffleBoard() {

        ShuffleboardTab tab = Shuffleboard.getTab("Test");

        // Limelight
        // HttpCamera httpCamera = new HttpCamera("Limelight",
        // "http://limelight.local:5800");
        // CameraServer.addCamera(httpCamera);
        // tab.add(httpCamera).withPosition(7, 0).withSize(3, 2);

        // New List Layout
        // ShuffleboardContainer pos = tab.getLayout("Position", "List
        // Layout").withPosition(0, 0).withSize(2, 3);

        // Field
        tab.add(drivetrain.getField()).withPosition(2, 1).withSize(5, 3);

        // Modes
        // tab.addBoolean("Slow Mode", () -> isSlow()).withPosition(2, 0).withSize(2,
        // 1);
        // tab.addBoolean("Roll Mode", () -> isRoll()).withPosition(5, 0).withSize(1,
        // 1);

        // Robot (Reverse order for list layout)
        // pos.addDouble("Robot R", () -> drivetrain.getRobotR())
        // .withWidget("Gyro");
        // ;
        tab.addDouble("Robot Y", () -> drivetrain.getRobotY());
        // .withWidget("Number Bar");
        tab.addDouble("Robot X", () -> drivetrain.getRobotX());

        // tab.addBoolean("IN RANGE", () -> drivetrain.robotNearHP());

        tab.addBoolean("FAST MODE", () -> {
            return isTurbo;
        });

        tab.addDouble("LEFTCR", () -> drivetrain.getForwardRangeLeft());
        tab.addDouble("RIGHTCR", () -> drivetrain.getForwardRangeRight());


        // tab.add("Auto Chooser", autoChooser);

    }

    private void configureBindings() {

        // STICK MOVEMENT
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive
                        // .withVelocityX(-driveController.getLeftY() * Constants.MAX_SPEED *
                        // getAxisMovementScale())
                        // .withVelocityY(-driveController.getLeftX() * Constants.MAX_SPEED *
                        // getAxisMovementScale())
                        .withVelocityX(-driveController.getLeftY() * (isTurbo ? MaxSpeedTurbo : MaxSpeed)
                                * getAxisMovementScale() //* elevatorSlowSpeed()
                                )
                        .withVelocityY(-driveController.getLeftX() * (isTurbo ? MaxSpeedTurbo : MaxSpeed)
                                * getAxisMovementScale() //* elevatorSlowSpeed()
                                )
                        .withRotationalRate(-driveController.getRightX() * MaxAngularRate * getAxisMovementScale())));

        driveController.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric())); // RESET POSE

        driveController.leftBumper().whileTrue(elevatorSubsystem.runOnce(() -> elevatorSubsystem.lower()));
        driveController.rightBumper().whileTrue(elevatorSubsystem.runOnce(() -> elevatorSubsystem.raise(manipulatorSubsystem)));

        operatorController.leftBumper().whileTrue(elevatorSubsystem.runOnce(() -> elevatorSubsystem.lower()));
        operatorController.rightBumper().whileTrue(elevatorSubsystem.runOnce(() -> elevatorSubsystem.raise(manipulatorSubsystem)));

        driveController.a().onTrue(new SequentialCommandGroup(
                new InstantCommand(() -> {
                    int nearestTag = VisionSubsystem.getNearestTag();
                    Target nearestTarget = new Target(this);
                    nearestTarget.setLocation(new Target.Location(nearestTag, Side.LEFT)); // Use forceTag constructor
                    nearestTarget.setHeight(Target.Height.BRANCH_L4); // Not used here but required
                    assistiveEAPose = nearestTarget.getTagRelativePose();
                }),
                new ExactAlign(drivetrain, assistiveEAPose)));

        driveController.b().onTrue(new SequentialCommandGroup(
                new InstantCommand(() -> {
                    int nearestTag = VisionSubsystem.getNearestTag();
                    Target nearestTarget = new Target(this);
                    nearestTarget.setLocation(new Target.Location(nearestTag, Side.RIGHT)); // Use forceTag constructor
                    nearestTarget.setHeight(Target.Height.BRANCH_L4); // Not used here but required
                    assistiveEAPose = nearestTarget.getTagRelativePose();
                }),
                new ExactAlign(drivetrain, assistiveEAPose)));

        /////////////////////////////////////////////////////////
        
        operatorController.y().onTrue(manipulatorSubsystem.load());

        // operatorController.povLeft().onTrue(new ScoreCommand(SF2Constants.SETPOINT_REEF_T2, elevatorSubsystem, manipulatorSubsystem));
        // operatorController.povUp().onTrue(new ScoreCommand(SF2Constants.SETPOINT_REEF_T3, elevatorSubsystem, manipulatorSubsystem));
        // operatorController.povRight().onTrue(new ScoreCommand(SF2Constants.SETPOINT_REEF_T4, elevatorSubsystem, manipulatorSubsystem));

        operatorController.povLeft().onTrue(new ElevatorCommand(elevatorSubsystem, manipulatorSubsystem, SF2Constants.SETPOINT_REEF_T2.getElevator(), true));
        operatorController.povUp().onTrue(new ElevatorCommand(elevatorSubsystem, manipulatorSubsystem, SF2Constants.SETPOINT_REEF_T3.getElevator(), true));
        operatorController.povRight().onTrue(new ElevatorCommand(elevatorSubsystem, manipulatorSubsystem, SF2Constants.SETPOINT_REEF_T4.getElevator(), true));

        operatorController.b().onTrue(new EjectCommand(manipulatorSubsystem));
        operatorController.x().onTrue(manipulatorSubsystem.eject());

        operatorController.a().onTrue(new ElevatorCommand(elevatorSubsystem, manipulatorSubsystem, SF2Constants.SETPOINT_MIN.getElevator(), true));


        //driveController.start().onTrue(armSubsystem.runOnce(() -> armSubsystem.zeroArm()));

        // driveController.start().onTrue(drivetrain.getPathfinderCommand(atf, target1));

        //driveController.a().onTrue(drivetrain.runOnce(() -> moveToSetpoint(SF2Constants.SETPOINT_ALGAE_T2))); // RESET POSE
        //driveController.b().onTrue(drivetrain.runOnce(() -> moveToSetpoint(SF2Constants.SETPOINT_ALGAE_T3))); // RESET POSE

        // // driveController.x().onTrue(armSubsystem.runOnce(() -> {
            // // armSubsystem.setZero();
            // // }));
            // driveController.x().onTrue(drivetrain.runOnce(() -> moveToSetpoint(Constants.SETPOINT_PROCESSOR)));
            
        // driveController.povLeft().onTrue(new InstantCommand(() -> {isTurbo = !isTurbo;}));

        // TagRelativePose testingTagRelativePose = new TagRelativePose(21, 0.0 // 0.67
        // , 0.142, 0.0); // x - f/b     y = l/r
        // driveController.y().onTrue(new ExactAlign(drivetrain, testingTagRelativePose));

    //     driveController.x().onTrue(new ExactAlign(drivetrain, target1.getTagRelativePose()));
    //     driveController.a().onTrue(new InstantCommand(() -> pathfinderSubsystem.queueFind(new Location(Landmark.STATION_RIGHT))));
    //     driveController.b().onTrue(new InstantCommand(() -> pathfinderSubsystem.queueFind(new Location(Landmark.REEF_FRONT_RIGHT))));
    //    driveController.b().onTrue(drivetrain.runOnce( () -> moveToSetpoint(SF2Constants.SETPOINT_REEF_T4))
    //     driveController.y().onTrue(new ExactAlign(drivetrain, target2.getTagRelativePose()));
        // driveController.x().onTrue(new InstantCommand(() -> System.out.println(target.getTagRelativePose())));

        // driveController.x().onTrue(new SequentialCommandGroup(
        //     new ExactAlign(drivetrain, target1.getTagRelativePose()),
        //     new ScoreCommand(target1.getSetpoint(), elevatorSubsystem, manipulatorSubsystem)
        // ));


        // driveController.povUp().onTrue(new CanRangeStation(drivetrain));

        
     


      //  driveController.b().onTrue(new InstantCommand(() -> target1.cycleLocationRight()));
      //  driveController.a().onTrue(new InstantCommand(() -> target1.cycleLocationLeft()));

        //driveController.x().onTrue(new TestCommand(drivetrain));

        // driveController.x().onTrue(new WristCommand(armSubsystem, SF2Constants.WRIST_MIN_POSITION, true));
        // driveController.y().onTrue(new WristCommand(armSubsystem, SF2Constants.WRIST_MAX_POSITION, true));

        // operatorController.a().onTrue(new IntakeCommand(manipulatorSubsystem));

        // Climber

        // driveController.povDown().onTrue(climberSubsystem.runOnce(() ->
        // climberSubsystem.servoLogic()));
        // driveController.povRight().onTrue(climberSubsystem.runOnce(() ->
        // climberSubsystem.stop()));
        // driveController.povLeft().onTrue(climberSubsystem.runOnce(() ->
        // climberSubsystem.speed(0.5)));
        // driveController.povUp().onTrue(climberSubsystem.runOnce(() ->
        // climberSubsystem.speed(-0.5)));

        // driveController.povDown().onTrue(elevatorSubsystem.runOnce(() -> elevatorSubsystem.setElevatorZero()));
        // driveController.povLeft().onTrue(elevatorSubsystem.runOnce(() -> elevatorSubsystem.lowerToLimit()));

        //operatorController.rightBumper().onTrue(armSubsystem.runOnce(() -> armSubsystem.armUp()));
        //operatorController.leftBumper().onTrue(armSubsystem.runOnce(() -> armSubsystem.armDown()));

        //operatorController.y().onTrue(new InstantCommand(() -> moveToSetpoint(SF2Constants.SETPOINT_BARGE)));




        // operatorController.a().onTrue(new InstantCommand(() -> manipulatorSubsystem.reverse()));
        // operatorController.b().onTrue(new EjectCommand(manipulatorSubsystem));
        // operatorController.x().onTrue(new InstantCommand(() -> manipulatorSubsystem.stop()));

        // operatorController.x().onTrue(manipulatorSubsystem.eject());

        // operatorController.y().onTrue(new SequentialCommandGroup(
        //     new ElevatorCommand(elevatorSubsystem, SF2Constants.SETPOINT_REEF_T4.getElevator(), true),
        //     new EjectCommand(manipulatorSubsystem),
        //     new ElevatorCommand(elevatorSubsystem, SF2Constants.SETPOINT_MIN.getElevator(), true)
        // ));




        // stop-style equivalents for above
        // operatorController.b().onFalse(new InstantCommand(() -> manipulatorSubsystem.stop()));
        // operatorController.a().onFalse(new InstantCommand(() -> manipulatorSubsystem.stop()));

        // X is used for allowing max outtake in core.periodic
        // operatorController.x().onTrue(new InstantCommand(() -> performRetract()));

        // operatorController.back().onTrue(armSubsystem.runOnce(() ->
        // armSubsystem.rotateWristIntake()));

        // operatorController.start().onTrue(new InstantCommand(() ->
        // pathfinderSubsystem.queueFind(new Location(Landmark.REEF_FRONT,
        // Side.LEFT))));
        // operatorController.back().onTrue(new InstantCommand(() ->
        // pathfinderSubsystem.queueAlign(Height.BRANCH_L3)));

        // operatorController.povDown().onTrue(new InstantCommand(() -> moveToSetpoint(SF2Constants.SETPOINT_REEF_T1)));
        // operatorController.povLeft().onTrue(new InstantCommand(() -> moveElevatorOnly(SF2Constants.SETPOINT_REEF_T2.getElevator())));
        // operatorController.povUp().onTrue(new InstantCommand(() -> moveElevatorOnly(SF2Constants.SETPOINT_REEF_T3.getElevator())));
        // operatorController.povRight().onTrue(new InstantCommand(() -> moveElevatorOnly(SF2Constants.SETPOINT_REEF_T4.getElevator())));

     

        //public SequentialCommandGroup GoTo(double setpoint) {


    }

    public void forwardAlign() {
    }

    public CommandSwerveDrivetrain getDrivetrain() {
        return drivetrain;
    }

    public PathfinderSubsystem getPathfinderSubsystem() {
        return pathfinderSubsystem;
    }

    // public NavInterfaceSubsystem getNavInterfaceSubsystem() {
    //     return navInterfaceSubsystem;
    // }

    public ManipulatorSubsystem getManipulatorSubsystem() {
        return manipulatorSubsystem;
    }

    public ElevatorSubsystem getElevatorSubsystem() {
        return elevatorSubsystem;
    }

    // public Command getAutonomousCommand() {
    // return autoChooser.getSelected();
    // }

    public void doPathfind(Pose2d target) {
        PathConstraints constraints = new PathConstraints(
                3, 4, // 3 - 4
                Units.degreesToRadians(540),
                Units.degreesToRadians(720));

        System.out.println(target);

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        pathfindingCommand = AutoBuilder.pathfindToPose(
                target,
                constraints,
                0);

        pathfindingCommand.schedule();

        System.out.println("PATHFIND TO " + target.toString() + " STARTED");
    }

    public void doPathfindToPath(String path) {
        try {

            PathPlannerPath pathData = PathPlannerPath.fromPathFile(path);

            PathConstraints constraints = new PathConstraints(
                    3, 4, // 3 - 4
                    Units.degreesToRadians(540),
                    Units.degreesToRadians(720));

            // Since AutoBuilder is configured, we can use it to build pathfinding commands
            pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                    pathData,
                    constraints);

            pathfindingCommand.schedule();

            System.out.println("PATHFIND TO " + path + " STARTED");

        } catch (Exception e) {
            DriverStation.reportError("Pathing failed: " + e.getMessage(), e.getStackTrace());
        }
    }

    public Command getPath(String id) {
        try {
            // Load the path you want to follow using its name in the GUI
            PathPlannerPath path = PathPlannerPath.fromPathFile(id);

            // Create a path following command using AutoBuilder. This will also trigger
            // event markers.
            return AutoBuilder.followPath(path);

        } catch (Exception e) {
            DriverStation.reportError("Pathing failed: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }

    public double getAxisMovementScale() {
        return (1 - (driveController.getRightTriggerAxis() * 0.85));
    }

    public double elevatorSlowSpeed() {
        if (currentElevatorPosition > 20) {
            return 20 / currentElevatorPosition;
        }

        return 1;
    }

    int clock = 0;

    public void corePeriodic() {
        
        // If either of our analog sticks are moved, we want to disable the auto
        if (driveController.getLeftX() != 0 || driveController.getLeftY() != 0) {
            pathfinderSubsystem.stopAll();
            if (autoCommandGroup != null) {
                autoCommandGroup.cancel();
            }
        }

        currentElevatorPosition = elevatorSubsystem.getElevatorPosition();
    }
}
