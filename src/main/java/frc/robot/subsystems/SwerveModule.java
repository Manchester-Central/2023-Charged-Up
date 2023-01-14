// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.text.Position;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Robot;

public class SwerveModule {
  private Translation2d m_translation;
  private double m_simdistance;
  private SwerveModuleState m_targetState;
  /** Creates a new SwerveModule. */
  public SwerveModule(Translation2d translation) {
    m_translation = translation;
    m_simdistance = 0;
    m_targetState = new SwerveModuleState(0,Rotation2d.fromDegrees(0));
  }

  public void setTarget(SwerveModuleState state) {
    m_targetState = state;
  }
  public SwerveModuleState getModuleState() {
    if (Robot.isSimulation()) {
      return m_targetState; 
    }
    return null; 
    // TODO
  }

  public Translation2d getTranslation() {
    return m_translation;
  }

  public SwerveModulePosition getPosition() {
    if (Robot.isSimulation()) {
      m_simdistance = m_simdistance+m_targetState.speedMetersPerSecond / Constants.UpdateFrequency_Hz;
      return new SwerveModulePosition(m_simdistance,m_targetState.angle);
     }
      return null; // TODO
  }

}
