// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDrive;

public class ResetHeading extends CommandBase {

  public enum Direction {
    Up,
    Down,
    Left,
    Right
  }

  /** Creates a new ResetHeading. */
  private SwerveDrive m_SwerveDrive;
  private Direction m_direction;
  public ResetHeading(SwerveDrive swervedrive, Direction direction) {
    m_SwerveDrive = swervedrive;
    m_direction = direction;
    addRequirements(swervedrive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    boolean BlueAlliance = (DriverStation.getAlliance() == Alliance.Blue);
    double heading = 0;
    switch (m_direction){
      case Up:
        heading = BlueAlliance ? 180 : 0;
        break;
      case Down:
        heading = BlueAlliance ? 0 : 180;
        break;
      case Left:
        heading = BlueAlliance ? 270 : 90;
        break;
      case Right:
        heading = BlueAlliance ? 90 : 270;
        break;
    }
    m_SwerveDrive.resetHeading(Rotation2d.fromDegrees(heading));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
