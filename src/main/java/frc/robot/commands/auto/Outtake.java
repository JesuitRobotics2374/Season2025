// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.OuttakeSubsystem;

public class Outtake extends Command {

  int counter = 0;

  OuttakeSubsystem outtake;

  /** Creates a new Outtake. */
  public Outtake(OuttakeSubsystem outtake) {
    this.outtake = outtake;

    addRequirements(outtake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    outtake.outtake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    counter++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    counter = 0;
    outtake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println(counter);
    return (counter > 30);
  }
}
