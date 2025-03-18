package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import java.security.InvalidParameterException;

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

    private static final double POSITION_0 = 2; // Lowest Height
    private static final double POSITION_1 = 25; // Position of the lowest reef level
    private static final double POSITION_2 = 50; // Position of lowest branch
    private static final double POSITION_3 = 75; // Position of middle branch
    private static final double POSITION_4 = 100; // Position of highest branch
    private static final double IH = 125; // Position of intake (number 5)
    private static final double AH = 125; // Position of algae outtake height (number 6)

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
        slot0Configs.kP = 0.5; // Controls the response to position error—how much the motor reacts to the
                               // difference between the current position and the target position.
        slot0Configs.kI = 0.01; // Addresses steady-state error, which occurs when the motor doesn’t quite reach
                                // the target position due to forces like gravity or friction.
        slot0Configs.kD = 0.1; // Responds to the rate of change of the error, damping the motion as the motor
                               // approaches the target. This reduces overshooting and oscillations.

        motionMagicConfigs.MotionMagicCruiseVelocity = 200; // Target velocity in rps
        motionMagicConfigs.MotionMagicAcceleration = 180; // Target acceleration in rps/s
        motionMagicConfigs.MotionMagicJerk = 1000; // Target jerk in rps/s/s

        elevatorMotor1.getConfigurator().apply(talonFXConfigs);
        elevatorMotor1.getConfigurator().apply(slot0Configs);
        elevatorMotor1.getConfigurator().apply(motionMagicConfigs);
        // elevatorMotor1.getConfigurator().apply(magnetSensorConfigs);

        elevatorMotor2.setControl(new Follower(elevatorMotor1.getDeviceID(), true));

        double absPosition = shaftEncoder.getAbsolutePosition().getValueAsDouble() + elevatorCycleOnStart;

        ShuffleboardTab tab = Shuffleboard.getTab("Test");
        tab.addDouble("ABS STARTING", () -> {return absPosition;});
        tab.addDouble("ELE CURRENT", () -> {return elevatorMotor1.getPosition().getValueAsDouble();});

        // shaftEncoder.setPosition(0);
        elevatorMotor1.setPosition(absPosition * Constants.ELEVATOR_RATIO);
    }

    public void doEstimatedZero() {
        double absPosition = shaftEncoder.getAbsolutePosition().getValueAsDouble() + elevatorCycleOnStart;
        elevatorMotor1.setPosition(absPosition * Constants.ELEVATOR_RATIO);
    }

    public void setElevatorZero() {
        shaftEncoder.setPosition(0.0);
        elevatorMotor1.setPosition(0.0);

        MotionMagicVoltage m_request = new MotionMagicVoltage(0);
        elevatorMotor1.setControl(m_request);
    }

    public void zeroElevator() { // Actual code is in perodic
        zeroingElevator = true;
    }

    public void stopElevator() {
        double elevatorPosition = elevatorMotor1.getPosition().getValueAsDouble();

        MotionMagicVoltage m_request = new MotionMagicVoltage(elevatorPosition);
        elevatorMotor1.setControl(m_request);

        elevatorMotor1.stopMotor();
        elevatorMotor1.setNeutralMode(NeutralModeValue.Brake);
        currentlyMovingDown = false;
    }

    public void elevatorGoTo(int newPos) {
        double posGoTo = convertPos(newPos);
        if (posGoTo < elevatorMotor1.getPosition().getValueAsDouble()) {
            currentlyMovingDown = true;
        } else {
            currentlyMovingDown = false;
        }
        MotionMagicVoltage m_request = new MotionMagicVoltage(posGoTo);
        elevatorMotor1.setControl(m_request.withEnableFOC(true).withOverrideBrakeDurNeutral(true));
    }

    public void elevatorGoToDouble(double pos) {
        if (pos < elevatorMotor1.getPosition().getValueAsDouble()) {
            currentlyMovingDown = true;
        } else {
            currentlyMovingDown = false;
        }
        MotionMagicVoltage m_request = new MotionMagicVoltage(pos);
        elevatorMotor1.setControl(m_request.withEnableFOC(true).withOverrideBrakeDurNeutral(true));
    }

    public void lower() {
       // if (!limitSwitch.get()) {
            currentlyMovingDown = true;
            MotionMagicVoltage m_request = new MotionMagicVoltage(elevatorMotor1.getPosition().getValueAsDouble() - Constants.ELEVATOR_MOVE_AMOUNT);
            elevatorMotor1.setControl(m_request.withEnableFOC(true).withOverrideBrakeDurNeutral(true));
       // }
    }

    public void lower_to_limt() {
        // if (!limitSwitch.get()) {
             currentlyMovingDown = true;
             while (currentlyMovingDown) {
                MotionMagicVoltage m_request = new MotionMagicVoltage(elevatorMotor1.getPosition().getValueAsDouble() - Constants.ELEVATOR_MOVE_AMOUNT);
                elevatorMotor1.setControl(m_request.withEnableFOC(true).withOverrideBrakeDurNeutral(true));
                if (!limitSwitch.get() ) {
                    setElevatorZero();
                    currentlyMovingDown = false;
                }

             }

        // }
     }

    public void lower(double amount) {
      //  if (!limitSwitch.get()) {
            currentlyMovingDown = true;
            MotionMagicVoltage m_request = new MotionMagicVoltage(elevatorMotor1.getPosition().getValueAsDouble() - amount);
            elevatorMotor1.setControl(m_request.withEnableFOC(true).withOverrideBrakeDurNeutral(true));
      //  }
    }

    public void raise() {
        currentlyMovingDown = false;
        MotionMagicVoltage m_request = new MotionMagicVoltage(elevatorMotor1.getPosition().getValueAsDouble() + Constants.ELEVATOR_MOVE_AMOUNT);
        elevatorMotor1.setControl(m_request.withEnableFOC(true).withOverrideBrakeDurNeutral(true));

        // Since the new request is based on the current position, there is not stacking
        // of lower requests
        // The delta essentiallly is the speed of the lower
    }

    public void raise(double amount) {
        currentlyMovingDown = false;
        MotionMagicVoltage m_request = new MotionMagicVoltage(elevatorMotor1.getPosition().getValueAsDouble() + amount);
        elevatorMotor1.setControl(m_request.withEnableFOC(true).withOverrideBrakeDurNeutral(true));

        // Since the new request is based on the current position, there is not stacking
        // of lower requests
        // The delta essentiallly is the speed of the lower
    }

    private double convertPos(int heightLevel) { // Get values once elevator is finished
        if (heightLevel == 0) {
            return POSITION_0;
        } else if (heightLevel == 1) {
            return POSITION_1;
        } else if (heightLevel == 2) {
            return POSITION_2;
        } else if (heightLevel == 3) {
            return POSITION_3;
        } else if (heightLevel == 4) {
            return POSITION_4;
        } else if (heightLevel == 5) {
            return IH;
        } else if (heightLevel == 6) {
            return AH;
        } else
            throw new InvalidParameterException("Number is not in accepted height range");
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

        // elevatorMotor1.getSupplyCurrent().getValueAsDouble() > 0.8 // But from T4 to Min elevator uses 0.8 amps
        if (currentlyMovingDown && !limitSwitch.get()) { // limit is reversed
            setElevatorZero();
        }
        
    }

    public void changeBy(double d) {
        elevatorGoToDouble(d + elevatorMotor1.getPosition().getValueAsDouble());
    }

}
