// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDrive;

public class SetPoseFromLimelights extends CommandBase {
  private SwerveDrive m_swerveDrive;
  private MedianFilter m_xFilter = new MedianFilter(100);
  private MedianFilter m_yFilter = new MedianFilter(100);
  private boolean m_seenValue = false;
  private double m_calculatedX, m_calculatedY;

  /** Creates a new SetPoseFromLimelights. */
  public SetPoseFromLimelights(SwerveDrive swerveDrive) {
    m_swerveDrive = swerveDrive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_xFilter.reset();
    m_yFilter.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var limelightPose = m_swerveDrive.getPoseFromLimelights();
    if(limelightPose != null) {
      m_seenValue = true;
      m_calculatedX = m_xFilter.calculate(limelightPose.getX());
      m_calculatedY = m_yFilter.calculate(limelightPose.getY());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(m_seenValue) {
      m_swerveDrive.resetPose(new Pose2d(m_calculatedX, m_calculatedY, m_swerveDrive.getOdometryRotation()));
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
