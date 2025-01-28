package frc.robot.subsystems;

import java.io.ObjectInputFilter.Config;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CustomParamsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ConnectedMotorValue;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

    public static TalonFX elevatorMotor1;
    public static TalonFX elevatorMotor2;
    public static CANrange range;
    public CANcoderConfiguration coderConfig = new CANcoderConfiguration();
    private CANrangeConfiguration rangeConfig = new CANrangeConfiguration();

    private int heightPos;
    // PID and Motion Magic Constants - GET PROPER VALUES GET PROPER VALUES OFSIEJFSOIEJFOISEJFOISJFOISEJFOISJFEOISEJFOIJakapofewwajfoaiwejfo
    //aoiewfjawoifejaoifjaoifjawoijawoifjwaeoifjwaef/
    //aowifejawoifejawoijafewoijaoifeoiajfewhaoiugewhapougewposehgi
    private static final double kP = 0.1;  // Proportional gain
    private static final double kI = 0.0;  // Integral gain
    private static final double kD = 0.0;  // Derivative gain
    private static final double kF = 0.05; // Feedforward gain
    private static final int cruiseVelocity = 15000; // Encoder ticks per 100ms
    private static final int acceleration = 6000;    // Encoder ticks per 100ms^2

    // Elevator Positions (Encoder Ticks) - GET PROPER VALUES GET PROPER VALUES OFSIEJFSOIEJFOISEJFOISJFOISEJFOISJFEOISEJFOIJakapofewwajfoaiwejfo
    //aoiewfjawoifejaoifjaoifjawoijawoifjwaeoifjwaef/
    //aowifejawoifejawoijafewoijaoifeoiajfewhaoiugewhapougewposehgi
    private static final int POSITION_0 = 0;
    private static final int POSITION_1 = 25000;
    private static final int POSITION_2 = 50000;
    private static final int POSITION_3 = 75000;
    private static final int POSITION_4 = 100000;

    final MotionMagicVoltage m_request;

    // CANcoder beltPosition = new CANcoder(Constants.beltPosition_ID);

    // DigitalInput L1Bottom = new DigitalInput(0);
    // DigitalInput L1Top = new DigitalInput(1);
    // DigitalInput L2Bottom = new DigitalInput(2);
    // DigitalInput L2Top = new DigitalInput(3);
    // DigitalInput L3Bottom = new DigitalInput(4);
    // DigitalInput L3Top = new DigitalInput(5);

    // public CANcoderConfiguration withMagnetSensor(MagnetSensorConfigs newMagnetSensor) {
    //     //TODO???????
    // }

    //private final double elevatorSpeed = 0.250;  -> find appropriate value

    public ElevatorSubsystem() {
        elevatorMotor1 = new TalonFX(27); // TODO DEVICE ID
        elevatorMotor2 = new TalonFX(0); // TODO DEVICE ID
        coderConfig = new CANcoderConfiguration();


        heightPos = 0;
        
        var talonFXConfigs = new TalonFXConfiguration();
        var slot0Configs = talonFXConfigs.Slot0;

        slot0Configs.kG = 0.25; //output to overcome gravity TODO
        slot0Configs.kV = 0.12; //output per unit target velocity TODO
        slot0Configs.kA = 0.01; //output per unit target acceleration TODO
        slot0Configs.kP = 3; //output per unit of error in position TODO
        slot0Configs.kI = 0; //output per unit of integrated error in position TODO
        slot0Configs.kD = 0.1; //output per unit of error in velocity TODO

        var motionMagicConfigs = talonFXConfigs.MotionMagic;

        motionMagicConfigs.MotionMagicCruiseVelocity = 2; // Target velocity in rps TODO
        motionMagicConfigs.MotionMagicAcceleration = 4; // Target acceleration in rps/s TODO
        motionMagicConfigs.MotionMagicJerk = 40; // Target jerk in rps/s/s TODO
        
        elevatorMotor1.getConfigurator().apply(talonFXConfigs);
        elevatorMotor2.getConfigurator().apply(talonFXConfigs);

        m_request = new MotionMagicVoltage(1);
    }

    public void startElevatorUp() {
        //elevatorMotor1.set(elevatorSpeed);
        //elevatorMotor2.set(-1 * elevatorSpeed);

        //where elevatorMotor1 and elevatorMotor2 have inversed directions
    }

    public void startElevatorDown() {
        //elevatorMotor1.set(-1 * elevatorSpeed);
        //elevatorMotor2.set(elevatorSpeed);
    }

    public void stopElevator() {
        elevatorMotor1.stopMotor();
        elevatorMotor2.stopMotor();
    }

    public void testMM() {
        elevatorMotor1.setControl(m_request.withPosition(4));
    }

    public void elevatorGoTo(int pos) { //pos defines which height to go to. 0 is resting, 4 is max
        if (heightPos != pos) {


        }
    }

    // private int convertPos(int height) { //GET VALUES FOR THISISISISISISISI
    //     if (height == 0) {

    //     }
    //     else if (height == 1) {

    //     }
    //     else if (height == 2) {

    //     }
    //     else if (height == 3) {

    //     }
    //     else if (height == 4) {

    //     }
    // }

}
  