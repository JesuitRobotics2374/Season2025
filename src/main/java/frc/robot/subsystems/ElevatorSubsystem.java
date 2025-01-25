package frc.robot.subsystems;

import java.io.ObjectInputFilter.Config;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CustomParamsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ConnectedMotorValue;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class ElevatorSubsystem {

    public static TalonFX elevatorMotor1;
    public static TalonFX elevatorMotor2;
    public static CANrange range;
    public CANcoderConfiguration coderConfig = new CANcoderConfiguration();
    private CANrangeConfiguration rangeConfig = new CANrangeConfiguration();

    CANcoder beltPosition = new CANcoder(Constants.beltPosition_ID);

    DigitalInput L1Bottom = new DigitalInput(0);
    DigitalInput L1Top = new DigitalInput(1);
    DigitalInput L2Bottom = new DigitalInput(2);
    DigitalInput L2Top = new DigitalInput(3);
    DigitalInput L3Bottom = new DigitalInput(4);
    DigitalInput L3Top = new DigitalInput(5);

    public CANcoderConfiguration withMagnetSensor(MagnetSensorConfigs newMagnetSensor) {
    }

    //private final double elevatorSpeed = 0.250;  -> find appropriate value

    public ElevatorSubsystem() {
        elevatorMotor1 = new TalonFX(0); // TODO
        elevatorMotor2 = new TalonFX(0); // TODO
        coderConfig = new CANcoderConfiguration();
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

    
}
  