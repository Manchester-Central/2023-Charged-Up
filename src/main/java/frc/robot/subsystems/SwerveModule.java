// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
  private Translation2d m_translation;
  
  /** Creates a new SwerveModule. */
  public SwerveModule(Translation2d translation) {
    m_translation = translation;
  }
public void setTarget(SwerveModuleState state){
  
}

public Translation2d getTranslation() {
  return m_translation;
}

}
