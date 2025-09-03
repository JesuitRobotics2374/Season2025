package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

    public TalonFX elevatorMotor1;
    public TalonFX elevatorMotor2;
    public CANcoder shaftEncoder;
    private Pigeon2 pidgey;
    public DigitalInput limitSwitch;

    private boolean currentlyMovingDown = false;
    private boolean zeroingElevator = false;
    private State currentState = State.UNKNOWN;

    private boolean hasReachedLimit = false;

    private final double defaultSetpointMoveSpeed = 0.1;
    private final double defaultRaiseSpeed = 0.1;
    private final double defaultLowerSpeed = 0.1;
   
    private int elevatorCycleOnStart = 2;

    public ElevatorSubsystem() {

        this.elevatorMotor1 = new TalonFX(31, "FastFD");
        this.elevatorMotor2 = new TalonFX(32, "FastFD");
        this.pidgey = new Pigeon2(Constants.PIGEON_ID, "FastFD");
        this.shaftEncoder = new CANcoder(30, "FastFD");
        limitSwitch = new DigitalInput(1);

        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
        Slot0Configs slot0Configs = talonFXConfigs.Slot0;
        MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
        MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
        magnetSensorConfigs.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        slot0Configs.kG = 0.2; // Output of voltage to overcome gravity
        slot0Configs.kV = 0.1; // Output per unit target velocity, perhaps not needed
        slot0Configs.kA = 0.01; // Output per unit target acceleration, perhaps not needed
        slot0Configs.kP = 1.6; // Controls the response to position error—how much the motor reacts to the
                               // difference between the current position and the target position.
        slot0Configs.kI = 0.01; // Addresses steady-state error, which occurs when the motor doesn’t quite reach
                                // the target position due to forces like gravity or friction.
        slot0Configs.kD = 0.1; // Responds to the rate of change of the error, damping the motion as the motor
                               // approaches the target. This reduces overshooting and oscillations.

        motionMagicConfigs.MotionMagicCruiseVelocity = 360; // Target velocity in rps
        motionMagicConfigs.MotionMagicAcceleration = 380; // Target acceleration in rps/s
        motionMagicConfigs.MotionMagicJerk = 1500; // Target jerk in rps/s/s

        elevatorMotor1.getConfigurator().apply(talonFXConfigs);
        elevatorMotor1.getConfigurator().apply(slot0Configs);
        elevatorMotor1.getConfigurator().apply(motionMagicConfigs);
        // elevatorMotor1.getConfigurator().apply(magnetSensorConfigs);

        elevatorMotor2.setControl(new Follower(elevatorMotor1.getDeviceID(), true));

        double absPosition = shaftEncoder.getAbsolutePosition().getValueAsDouble() + elevatorCycleOnStart;

        ShuffleboardTab tab = Shuffleboard.getTab("Test");
        tab.addDouble("ABS STARTING", () -> {
            return absPosition;
        });
        tab.addDouble("ELE CURRENT", () -> {
            return elevatorMotor1.getPosition().getValueAsDouble();
        });

        // shaftEncoder.setPosition(0);
        elevatorMotor1.setPosition(absPosition * Constants.ELEVATOR_RATIO);
    }

    public enum State {
        MOVING_UP,
        MOVING_DOWN,
        ZEROES,
        UNKNOWN
    }

    public enum Height {
        TROUGH,
        BRANCH_L2,
        BRANCH_L3,
        BRANCH_L4,
    }

    public SequentialCommandGroup zeroPosition() {
        SequentialCommandGroup group = new SequentialCommandGroup();
        Command c = new InstantCommand(() -> zeroElevator());
        group.addCommands(c);

        return group;
    }

    public SequentialCommandGroup incrementUp(double amount) {
        SequentialCommandGroup group = new SequentialCommandGroup();
        Command c = new InstantCommand(() -> raise(amount));
        group.addCommands(c);

        return group;
    }

    public SequentialCommandGroup incrementDown(double amount) {
        SequentialCommandGroup group = new SequentialCommandGroup();
        Command c = new InstantCommand(() -> lower(amount));
        group.addCommands(c);

        return group;
    }

    public SequentialCommandGroup continuousUpStart(double mps) { // With non-parameter equivalent //TODO: THIS DOESNT WORK, IDK HOW TO DO UP
        if (true) return null;
        SequentialCommandGroup group = new SequentialCommandGroup();
        Command c = new InstantCommand(() -> zeroElevator());
        group.addCommands(c);

        return group;
    }

    public SequentialCommandGroup continuousUpStart() {
        return continuousUpStart(defaultRaiseSpeed);
    }                 

    public SequentialCommandGroup continuousDownStart(double mps) { // With non-parameter equivalent TODO: HOW IMPLEMENT THE PARAM
        SequentialCommandGroup group = new SequentialCommandGroup();
        Command c = new InstantCommand(() -> lowerToLimit());
        group.addCommands(c);

        return group;
    } 

    public SequentialCommandGroup continuousDownStart() {
        return continuousDownStart(defaultLowerSpeed);                         
    }

    public SequentialCommandGroup moveToSetpoint(Height height, double mps) { // With non-parameter (speed) equivalent TODO: HOW TO IMPLEMENT THE PARAM
        SequentialCommandGroup group = new SequentialCommandGroup();
        Command c = null;

        switch(height) {
            case TROUGH:
                c = new InstantCommand(() -> elevatorGoToDouble(12));
            case BRANCH_L2:
                c = new InstantCommand(() -> elevatorGoToDouble(0));
            case BRANCH_L3:
                c = new InstantCommand(() -> elevatorGoToDouble(37.76));
            case BRANCH_L4:
                c = new InstantCommand(() -> elevatorGoToDouble(97));
        }

        if (c == null) {
            return null;
        }

        group.addCommands(c);
        return group;
    } 

    public SequentialCommandGroup moveToSetpoint(Height height) {
        return moveToSetpoint(height, defaultSetpointMoveSpeed);
    }

    public SequentialCommandGroup haltAll() {
        SequentialCommandGroup group = new SequentialCommandGroup();
        Command c = new InstantCommand(() -> stopElevator());
        group.addCommands(c);

        return group;
    }

    public double mechanismHeight() {
        return getElevatorPosition();
    }

    public double mechanismSpeed() { //TODO: MAKE THIS WORK WHEN SPEED PARAMS WORK
        return 0;
    }

    public State mechanismStatus() { // MOVING_UP, MOVING_DOWN, ZEROED, UNKNOWN
        return currentState;
    } 

    public double getDefaultRaiseSpeed() { // Reference to private
        return defaultRaiseSpeed;
    }

    public double getDefaultLowerSpeed() { // Reference to private
        return defaultLowerSpeed;
    }








    private void setElevatorZero() {
        shaftEncoder.setPosition(0.0);
        elevatorMotor1.setPosition(0.0);

        MotionMagicVoltage m_request = new MotionMagicVoltage(0);
        elevatorMotor1.setControl(m_request);
    }

    private void zeroElevator() { // Actual code is in perodic
        zeroingElevator = true;
    }

    private void stopElevator() {
        double elevatorPosition = elevatorMotor1.getPosition().getValueAsDouble();

        MotionMagicVoltage m_request = new MotionMagicVoltage(elevatorPosition);
        elevatorMotor1.setControl(m_request);

        elevatorMotor1.stopMotor();
        elevatorMotor1.setNeutralMode(NeutralModeValue.Brake);
        currentlyMovingDown = false;
    }

    private void elevatorGoToDouble(double pos) {
        if (pos < elevatorMotor1.getPosition().getValueAsDouble()) {
            currentlyMovingDown = true;
        } else {
            currentlyMovingDown = false;
        }
        MotionMagicVoltage m_request = new MotionMagicVoltage(pos);
        elevatorMotor1.setControl(m_request.withEnableFOC(true).withOverrideBrakeDurNeutral(true));
    }

    private void lower() {
        if (hasReachedLimit) {
            System.out.println("Cancelled elevator move down: at bottom!");
            return;
        }
        // if (!limitSwitch.get()) {
        currentlyMovingDown = true;
        MotionMagicVoltage m_request = new MotionMagicVoltage(
                elevatorMotor1.getPosition().getValueAsDouble() - Constants.ELEVATOR_MOVE_AMOUNT);
        elevatorMotor1.setControl(m_request.withEnableFOC(true).withOverrideBrakeDurNeutral(true));
        // }
    }

    private void lowerToLimit() {
        // if (!limitSwitch.get()) {
        currentlyMovingDown = true;
        while (currentlyMovingDown) {
            MotionMagicVoltage m_request = new MotionMagicVoltage(
                    elevatorMotor1.getPosition().getValueAsDouble() - Constants.ELEVATOR_MOVE_AMOUNT);
            elevatorMotor1.setControl(m_request.withEnableFOC(true).withOverrideBrakeDurNeutral(true));
            if (!limitSwitch.get()) {
                setElevatorZero();
                currentlyMovingDown = false;
            }
        }

        // }
    }

    private void lower(double amount) {
        if (hasReachedLimit) {
            System.out.println("Cancelled elevator move down: at bottom!");
            return;
        }
        // if (!limitSwitch.get()) {
        currentlyMovingDown = true;
        MotionMagicVoltage m_request = new MotionMagicVoltage(elevatorMotor1.getPosition().getValueAsDouble() - amount);
        elevatorMotor1.setControl(m_request.withEnableFOC(true).withOverrideBrakeDurNeutral(true));
        // }
    }

    private void raise() {
        currentlyMovingDown = false;
        MotionMagicVoltage m_request = new MotionMagicVoltage(
                elevatorMotor1.getPosition().getValueAsDouble() + Constants.ELEVATOR_MOVE_AMOUNT);
        elevatorMotor1.setControl(m_request.withEnableFOC(true).withOverrideBrakeDurNeutral(true));

        // Since the new request is based on the current position, there is not stacking
        // of lower requests
        // The delta essentiallly is the speed of the lower
    }

    private void raise(double amount) {
        currentlyMovingDown = false;
        MotionMagicVoltage m_request = new MotionMagicVoltage(elevatorMotor1.getPosition().getValueAsDouble() + amount);
        elevatorMotor1.setControl(m_request.withEnableFOC(true).withOverrideBrakeDurNeutral(true));

        // Since the new request is based on the current position, there is not stacking
        // of lower requests
        // The delta essentiallly is the speed of the lower
    }

    private double getElevatorPosition() {
        return elevatorMotor1.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        // Robot tilting
        if (pidgey.getRotation3d().getMeasureX().abs(Degrees) > Constants.MAX_TIP_ANGLE
                || pidgey.getRotation3d().getMeasureY().abs(Degrees) > Constants.MAX_TIP_ANGLE) {
            lower();
            System.out.println("Lowering Elevator Due To Tipping");
        }

        if (zeroingElevator) {
            System.out.println("Zeroing Elevator");
            if (limitSwitch.get()) {
                zeroingElevator = false;
                setElevatorZero();
            } else {
                lower(1.5 * Constants.ELEVATOR_MOVE_AMOUNT);
            }
        }

        // elevatorMotor1.getSupplyCurrent().getValueAsDouble() > 0.8 // But from T4 to
        // Min elevator uses 0.8 amps
        if (currentlyMovingDown && !limitSwitch.get() && !hasReachedLimit) { // limit is reversed
            setElevatorZero();
            hasReachedLimit = true;
            currentlyMovingDown = false;
        }

        if (hasReachedLimit && limitSwitch.get()) {
            hasReachedLimit = false;
        }

    }

    public void changeBy(double d) {
        elevatorGoToDouble(d + elevatorMotor1.getPosition().getValueAsDouble());
    }

    public SequentialCommandGroup GoTo(double setpoint) {
        SequentialCommandGroup group = new SequentialCommandGroup();
        Command c = new InstantCommand(() -> this.elevatorGoToDouble(setpoint));
        group.addCommands(c);

        return group;
    }

}
