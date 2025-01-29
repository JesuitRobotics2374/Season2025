package frc.robot.subsystems;

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

    private int currentPos; //Height of the current elevator position, perhaps removable with absolute encoder positions

    private static final int POSITION_0 = 0; //Resting Height
    private static final int POSITION_2 = 0; //Position of lowest branch
    private static final int POSITION_3 = 0; //Position of middle branch
    private static final int POSITION_4 = 0; //Position of highest branch
    private static final int IH = 0; //Position of intake (also number 1)

    private MotionMagicVoltage m_request; //The magic motion request, will change

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


        currentPos = 0;
        
        var talonFXConfigs = new TalonFXConfiguration();
        var slot0Configs = talonFXConfigs.Slot0;

        //NEED CONFIGURING (ONCE ELEVATOR IS COMPLETE):
        slot0Configs.kG = 0.00; //Output of voltage to overcome gravity
        slot0Configs.kV = 0.12; //output per unit target velocity, perhaps not needed
        slot0Configs.kA = 0.01; //output per unit target acceleration, perhaps not needed
        slot0Configs.kP = 0.1; //Controls the response to position error—how much the motor reacts to the difference between the current position and the target position.
        slot0Configs.kI = 0.00; //Addresses steady-state error, which occurs when the motor doesn’t quite reach the target position due to forces like gravity or friction.
        slot0Configs.kD = 0.00; //Responds to the rate of change of the error, damping the motion as the motor approaches the target. This reduces overshooting and oscillations.

        var motionMagicConfigs = talonFXConfigs.MotionMagic;

        motionMagicConfigs.MotionMagicCruiseVelocity = 2; // Target velocity in rps TODO
        motionMagicConfigs.MotionMagicAcceleration = 2; // Target acceleration in rps/s TODO
        motionMagicConfigs.MotionMagicJerk = 20; // Target jerk in rps/s/s TODO
        
        elevatorMotor1.getConfigurator().apply(talonFXConfigs);
        elevatorMotor2.getConfigurator().apply(talonFXConfigs);

       m_request = new MotionMagicVoltage(0);
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
        elevatorMotor2.stopMotor();
    }

    public void testMM() {
        elevatorMotor1.setControl(m_request.withPosition(10));
        System.out.println("What is the next step of the operation?");
        // elevatorMotor1.setControl(m_request.withPosition(0));
        // System.out.println("Insert locked in alien here");
        // elevatorMotor1.setControl(m_request.withPosition(5));
        // System.out.println("Insert explosion");
    }

    public void elevatorGoTo(int newPos) { //pos defines which height to go to. 0 is resting, 4 is max
        if (newPos < 0 || newPos < 4) {
            throw new InvalidParameterException("Number is not in accepted height range");
        }

        int levDiff = currentPos - convertPos(newPos);

        elevatorMotor1.setControl(m_request.withPosition(levDiff));
        elevatorMotor2.setControl(m_request.withPosition(levDiff));
        //FIGURE OUT IF WITHPOS IS A RELATIVE CHANGE OR AN ABSOLUTE CHANGE OIESFJOIESJFOISEJFOISEJFOISJFEOISJFEOISJFEOIJ
        //OIFESJOIESFOIESF:OIFESJO:IFESJ:OIFESJ:OSIEJF
        //OSIFEJOIESFJOIESFJOIESFOIESFJOSIEJFOISEJF

        currentPos = convertPos(newPos);
    }

    private int convertPos(int heightLevel) { //Get values once elevator is finished
        if (heightLevel == 0) {
            return POSITION_0;
        }
        else if (heightLevel == 1) {
            return IH;
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
        else throw new InvalidParameterException("Number is not in accepted height range");
    }

}
  