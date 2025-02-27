package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

    public TalonFX climbMotor;
    public CANcoder climbCoder;
    public Servo servo;
    public boolean canClimb = true;

    public ClimberSubsystem() {
        this.servo = new Servo(1);
        this.climbMotor = new TalonFX(27, "FastFD");
        this.climbCoder = new CANcoder(26, "FastFD");

        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
        Slot0Configs slot0Configs = talonFXConfigs.Slot0;
        MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;

        slot0Configs.kG = 0; // Output of voltage to overcome gravity
        slot0Configs.kV = 0.1; // Output per unit target velocity, perhaps not needed
        slot0Configs.kA = 0; // Output per unit target acceleration, perhaps not needed
        slot0Configs.kP = 0; // Controls the response to position error—how much the motor reacts to the
                             // difference between the current position and the target position.
        slot0Configs.kI = 0; // Addresses steady-state error, which occurs when the motor doesn’t quite reach
                             // the target position due to forces like gravity or friction.
        slot0Configs.kD = 0; // Responds to the rate of change of the error, damping the motion as the motor
                             // approaches the target. This reduces overshooting and oscillations.

        motionMagicConfigs.MotionMagicCruiseVelocity = 25; // Target velocity in rps
        motionMagicConfigs.MotionMagicAcceleration = 100; // Target acceleration in rps/s
        motionMagicConfigs.MotionMagicJerk = 500; // Target jerk in rps/s/s

        climbMotor.getConfigurator().apply(talonFXConfigs);
        climbMotor.getConfigurator().apply(slot0Configs);
        climbMotor.getConfigurator().apply(motionMagicConfigs);

        climbMotor.setPosition(climbCoder.getPosition().getValueAsDouble() * 100);

        climbMotor.setNeutralMode(NeutralModeValue.Brake);

    }

    public void speed(double s) {
        if(canClimb == true && s>0){
            climbMotor.set(s);
        }
        if(canClimb == false && s<0){
            climbMotor.set(s);
        }
        
    }

    public void servoPosition(double p) {
        servo.setPosition(p);
    }
    
    public void servoLogic(){
        if(canClimb == false){
            servoPosition(10);
            canClimb=true;
        }
        else if (canClimb == true){
            servoPosition(0);
            canClimb = false;
        }
    }

    public void stop() {
        climbMotor.stopMotor();
        climbMotor.set(0);
    }

    public void zeroSystem() {
        climbCoder.setPosition(0.0);
        climbMotor.setPosition(0.0);

        MotionMagicVoltage m_request = new MotionMagicVoltage(0);
        climbMotor.setControl(m_request);
    }

    int clock = 0;

    @Override

    public void periodic() {
        clock++;

        if (clock == 20) {
            // System.out.println("Motor: " + climbMotor.getPosition().getValueAsDouble());
            // System.out.println("Encoder: " + climbCoder.getPosition().getValueAsDouble());
            clock = 0;
        }

    }
}