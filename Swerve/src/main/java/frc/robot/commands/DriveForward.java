// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Encoder;

public class DriveForward extends CommandBase {

  private Drivetrain m_drivetrain;
  private Encoder m_encoder;
  private ChassisSpeeds m_speeds = new ChassisSpeeds(0.5, 0, 0);

  /** Creates a new DriveForward. */
  public DriveForward(Drivetrain drivetrain, Encoder encoder) {

    m_drivetrain = drivetrain;
    m_encoder = encoder;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("Init Forward Encoder", m_encoder.getDistance());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.drive(m_speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putNumber("End Forward Encoder", m_encoder.getDistance());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
