// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Encoder extends CommandBase {
  
  private DutyCycleEncoder m_encoder = new DutyCycleEncoder(9);

  /** Creates a new Encoder. */
  public Encoder() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public double getDistance() {
    return m_encoder.getDistance();
  }

  public boolean getConnected() {
    return m_encoder.isConnected();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
