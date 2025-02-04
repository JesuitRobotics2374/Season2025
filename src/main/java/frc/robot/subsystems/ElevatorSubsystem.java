package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.io.ObjectInputFilter.Config;
import java.security.InvalidParameterException;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CustomParamsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ConnectedMotorValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

    public TalonFX elevatorMotor1;
    public TalonFX elevatorMotor2;
    // public CANrange range;

    public CANcoder shaftEncoder;
    
    // public CANcoderConfiguration coderConfig = new CANcoderConfiguration();
    // private CANrangeConfiguration rangeConfig = new CANrangeConfiguration();

    //CANcoder beltPosition = new CANcoder(Constants.beltPosition_ID);

    private static final double POSITION_0 = 0; //Lowest Height
    private static final double POSITION_1 = 10; //Position of the lowest reef level
    private static final double POSITION_2 = 20; //Position of lowest branch
    private static final double POSITION_3 = 30; //Position of middle branch 
    private static final double POSITION_4 = 40; //Position of highest branch
    private static final double IH = 2.5; //Position of intake (also number 5)

    private MotionMagicConfigs motionMagicConfigs;

    // CANcoder beltPosition = new CANcoder(Constants.beltPosition_ID);

    DigitalInput L1Bottom = new DigitalInput(0);
    DigitalInput L1Top = new DigitalInput(1);
    DigitalInput L2Bottom = new DigitalInput(2);
    DigitalInput L2Top = new DigitalInput(3);
    DigitalInput L3Bottom = new DigitalInput(4);
    DigitalInput L3Top = new DigitalInput(5);

    private final double elevatorSpeed = 0.5;

    // public CANcoderConfiguration withMagnetSensor(MagnetSensorConfigs newMagnetSensor) {
    // 
    // }

   //public CANcoderConfiguration withMagnetSensor(MagnetSensorConfigs newMagnetSensor) {
    //}


    public ElevatorSubsystem() {
        this.elevatorMotor1 = new TalonFX(27, "FastFD"); // TODO DEVICE ID
        this.elevatorMotor2 = new TalonFX(28, "FastFD"); // TODO DEVICE ID

        //shaftEncoder = new CANcoder(35, "FastFD");
        // coderConfig = new CANcoderConfiguration();


        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
        Slot0Configs slot0Configs = talonFXConfigs.Slot0;
        motionMagicConfigs = talonFXConfigs.MotionMagic;

        // talonFXConfigs.Feedback.FeedbackRemoteSensorID = 35;
        // talonFXConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        // talonFXConfigs.Feedback.RotorToSensorRatio = 1/20;

        slot0Configs.kG = 0.00; //Output of voltage to overcome gravity
        slot0Configs.kV = 0.01; //Output per unit target velocity, perhaps not needed
        slot0Configs.kA = 0.01; //Output per unit target acceleration, perhaps not needed
        slot0Configs.kP = 1; //Controls the response to position error—how much the motor reacts to the difference between the current position and the target position.
        slot0Configs.kI = 0.01; //Addresses steady-state error, which occurs when the motor doesn’t quite reach the target position due to forces like gravity or friction.
        slot0Configs.kD = 0.01; //Responds to the rate of change of the error, damping the motion as the motor approaches the target. This reduces overshooting and oscillations.

        motionMagicConfigs.MotionMagicCruiseVelocity = 50; // Target velocity in rps
        motionMagicConfigs.MotionMagicAcceleration = 100; // Target acceleration in rps/s
        motionMagicConfigs.MotionMagicJerk = 1000; // Target jerk in rps/s/s

        elevatorMotor1.getConfigurator().apply(talonFXConfigs);
        elevatorMotor1.getConfigurator().apply(slot0Configs);
        elevatorMotor1.getConfigurator().apply(motionMagicConfigs);

        // elevatorMotor1.setNeutralMode(NeutralModeValue.Coast);

        elevatorMotor2.setControl(new Follower(elevatorMotor1.getDeviceID(), true));
    }

    public void upReset() {
        while (!L3Top.get()) {
            elevatorMotor1.set(elevatorSpeed);
        }
        stopElevator();
        zeroSystem();

    }

    public void zeroSystem() {
        MotionMagicVoltage m_request = new MotionMagicVoltage(0);

        elevatorMotor1.setPosition(0.0);
        elevatorMotor1.setControl(m_request);
    }

    public void downReset() { //elevator 1 and 2 have different directions to move, figure out which is which
        while (!L1Bottom.get()) {
            elevatorMotor1.set(-elevatorSpeed);
        }
        stopElevator();
        zeroSystem();
    }

    public void stopElevator() {
        elevatorMotor1.stopMotor();
    }

    public void testMM() {
        // MotionMagicVoltage voltage = new MotionMagicVoltage(10);
        // elevatorMotor1.setControl(voltage);
        System.out.println("--- Test Executed ---");
        System.out.println();
        MotionMagicVoltage m_request = new MotionMagicVoltage(100);
        elevatorMotor1.setControl(m_request);
        // elevatorMotor1.setControl(m_request.withPosition(0));
        // System.out.println("Insert locked in alien here");
        // elevatorMotor1.setControl(m_request.withPosition(5));
        // System.out.println("Insert explosion");
    }

    public void testMM2() {
        // MotionMagicVoltage voltage = new MotionMagicVoltage(10);
        // elevatorMotor1.setControl(voltage);
        System.out.println("--- Test Executed ---");
        System.out.println();
        MotionMagicVoltage m_request = new MotionMagicVoltage(0);
        elevatorMotor1.setControl(m_request);
        // elevatorMotor1.setControl(m_request.withPosition(0));
        // System.out.println("Insert locked in alien here");
        // elevatorMotor1.setControl(m_request.withPosition(5));
        // System.out.println("Insert explosion");
    }

    public void testMM3() {
        System.out.println("--- Test Executed ---");
        System.out.println();
        MotionMagicVoltage m_request = new MotionMagicVoltage(50);
        elevatorMotor1.setControl(m_request);
    }

    public void elevatorGoTo(int newPos) { //pos defines which height to go to. 0 is resting, 4 is top level, 5 is intake
        double posGoTo = convertPos(newPos);

        MotionMagicVoltage m_request = new MotionMagicVoltage(posGoTo);
        elevatorMotor1.setControl(m_request);
    }

    private double convertPos(int heightLevel) { //Get values once elevator is finished
        if (heightLevel == 0) {
            return POSITION_0;
        }
        else if (heightLevel == 1) {
            return POSITION_1;
        }
        else if (heightLevel == 2) {
            return POSITION_2;
        }
        else if (heightLevel == 3) {
            return POSITION_3;
        }
        else if (heightLevel == 4) {
            return POSITION_4;
        }
        else if (heightLevel == 5) {
            return IH;
        }
        else throw new InvalidParameterException("Number is not in accepted height range");
    }


    private int clock = 0;

    @Override
    public void periodic() {

        clock++;

        // if (L1Bottom.get()) {
        //     stopElevator();
        //     elevatorMotor1.setPosition(0);
        // }
        // if (L3Top.get()) { //GET THE VLSAUES FOR THIS
        //     stopElevator();
        //     elevatorMotor1.setPosition(0);
        // }

        if (clock == 20) {
            System.out.println(elevatorMotor1.getPosition());
            clock = 0;
        }
    }

}
  