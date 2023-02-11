// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class SwerveTune extends CommandBase {
  /** Creates a new SwerveTune. */
  private SwerveDrive m_swervedrive;
  public SwerveTune(SwerveDrive swervedrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swervedrive = swervedrive;
    addRequirements(swervedrive);
    SmartDashboard.putNumber("SwerveTune/Velocity", 0);
    SmartDashboard.putNumber("SwerveTune/Angle", 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double velocity = SmartDashboard.getNumber("SwerveTune/Velocity", 0); 
    double angle = SmartDashboard.getNumber("SwerveTune/Angle", 0); 
    var swerveModuleState = new SwerveModuleState(velocity, Rotation2d.fromDegrees(angle));
    m_swervedrive.debug_setSwerveModule(swerveModuleState);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
