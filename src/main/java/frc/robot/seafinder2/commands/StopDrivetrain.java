package frc.robot.seafinder2.commands;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class StopDrivetrain extends Command {
    private CommandSwerveDrivetrain drivetrain;

    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();

    public StopDrivetrain(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {

        System.out.println("STOPPING DRIVETRAIN");
        drivetrain.configNeutralMode(NeutralModeValue.Brake);
        drivetrain.setControl(driveRequest.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(0.0));
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
