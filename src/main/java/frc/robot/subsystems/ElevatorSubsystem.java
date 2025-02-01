package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.io.ObjectInputFilter.Config;
import java.security.InvalidParameterException;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CustomParamsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ConnectedMotorValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

    public TalonFX elevatorMotor1;
    // public TalonFX elevatorMotor2;
    // public CANrange range;

    public CANcoder shaftEncoder;
    
    // public CANcoderConfiguration coderConfig = new CANcoderConfiguration();
    // private CANrangeConfiguration rangeConfig = new CANrangeConfiguration();

    //CANcoder beltPosition = new CANcoder(Constants.beltPosition_ID);


    private double currentPos; //Height of the current elevator position, perhaps ret2ovable with absolute encoder positions

    private static final int POSITION_0 = 0; //Resting Height
    private static final int POSITION_1 = 0; //Position of the lowest reef
    private static final int POSITION_2 = 0; //Position of lowest branch
    private static final int POSITION_3 = 0; //Position of middle branch 
    private static final int POSITION_4 = 0; //Position of highest branch
    private static final int IH = 0; //Position of intake (also number 1)

    private MotionMagicConfigs motionMagicConfigs;


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

   //public CANcoderConfiguration withMagnetSensor(MagnetSensorConfigs newMagnetSensor) {
    //}


    //private final double elevatorSpeed = 0.250;  -> find appropriate value

    public ElevatorSubsystem() {
        this.elevatorMotor1 = new TalonFX(27, "FastFD"); // TODO DEVICE ID
        // this.elevatorMotor2 = new TalonFX(0); // TODO DEVICE ID

        shaftEncoder = new CANcoder(35, "FastFD");
        // coderConfig = new CANcoderConfiguration();


        currentPos = 0;
        var talonFXConfigs = new TalonFXConfiguration();
        var slot0Configs = talonFXConfigs.Slot0;

        final double intrinsicModifier = 1;

        //NEED CONFIGURING (ONCE ELEVATOR IS COMPLETE):
        slot0Configs.kG = 0.01 / intrinsicModifier; //Output of voltage to overcome gravity
        slot0Configs.kV = 0.12 / intrinsicModifier; //output per unit target velocity, perhaps not needed
        slot0Configs.kA = 0.01 / intrinsicModifier; //output per unit target acceleration, perhaps not needed
        slot0Configs.kP = 0.01 / intrinsicModifier; //Controls the response to position error—how much the motor reacts to the difference between the current position and the target position.
        slot0Configs.kI = 0.01 / intrinsicModifier; //Addresses steady-state error, which occurs when the motor doesn’t quite reach the target position due to forces like gravity or friction.
        slot0Configs.kD = 0.01 / intrinsicModifier; //Responds to the rate of change of the error, damping the motion as the motor approaches the target. This reduces overshooting and oscillations.

        motionMagicConfigs = talonFXConfigs.MotionMagic;

        motionMagicConfigs.MotionMagicCruiseVelocity = 40; // Target velocity in rps
        motionMagicConfigs.MotionMagicAcceleration = 200; // Target acceleration in rps/s
        motionMagicConfigs.MotionMagicJerk = 2000; // Target jerk in rps/s/s

        // talonFXConfigs.Feedback.FeedbackRemoteSensorID = 35;
        // talonFXConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        // talonFXConfigs.Feedback.RotorToSensorRatio = 20;

        this.elevatorMotor1.getConfigurator().apply(talonFXConfigs);
        // this.elevatorMotor1.getConfigurator().apply(slot0Configs);
        // this.elevatorMotor2.getConfigurator().apply(talonFXConfigs);
    }

    // public void startElevatorUp() {
    //     //elevatorMotor1.set(elevatorSpeed);
    //     //elevatorMotor2.set(-1 * elevatorSpeed);

    //     //where elevatorMotor1 and elevatorMotor2 have inversed directions
    // }

    // public void startElevatorDown() {
    //     //elevatorMotor1.set(-1 * elevatorSpeed);
    //     //elevatorMotor2.set(elevatorSpeed);
    // }

    public void stopElevator() {
        elevatorMotor1.stopMotor();
        // elevatorMotor2.stopMotor();
    }

    public void testMM() {
        // MotionMagicVoltage voltage = new MotionMagicVoltage(10);
        // elevatorMotor1.setControl(voltage);
        System.out.println("--- Test Executed ---");
        System.out.println();
        MotionMagicVoltage m_request = new MotionMagicVoltage(20);
        elevatorMotor1.setControl(m_request);
        // elevatorMotor1.setControl(m_request.withPosition(0));
        // System.out.println("Insert locked in alien here");
        // elevatorMotor1.setControl(m_request.withPosition(5));
        // System.out.println("Insert explosion");
    }

    public void elevatorGoTo(int newPos) { //pos defines which height to go to. 0 is resting, 4 is max
        if (newPos < 0 || newPos > 5) {
            throw new InvalidParameterException("Number is not in accepted height range");
        }

        double levDiff = currentPos - convertPos(newPos);

        // elevatorMotor1.setControl(m_request.withPosition(levDiff));
        // elevatorMotor2.setControl(m_request.withPosition(levDiff));
        //FIGURE OUT IF WITHPOS IS A RELATIVE CHANGE OR AN ABSOLUTE CHANGE

        currentPos = convertPos(newPos);
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

        if (clock == 25) {
            clock = 0;
            System.out.println("shaft out: " + shaftEncoder.getPosition());
            System.out.println("motor out: " + elevatorMotor1.getPosition());
            System.out.println("rotor out: " + elevatorMotor1.getRotorPosition());
        }

    }

}
  