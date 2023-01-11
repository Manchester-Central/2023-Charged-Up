// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {

  private SwerveModule m_frontLeft;
  private SwerveModule m_frontRight;
  private SwerveModule m_backLeft;
  private SwerveModule m_backRight;

  private SwerveDriveKinematics m_kinematics;

  /** Creates a new SwerveDrive. */
  public SwerveDrive() {

    m_frontLeft = new SwerveModule();
    m_frontRight = new SwerveModule();
    m_backLeft = new SwerveModule();
    m_backRight = new SwerveModule();
  }

  public void move(ChassisSpeeds chassisSpeeds){
  SwerveModuleState[] states=m_kinematics.toSwerveModuleStates(chassisSpeeds);
  m_frontLeft.setTarget(states[0]);
  //Todo 
  }
  public void moveDriverRelative(double x,double y,double omega){
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(y, -x, omega, getRotation());
    move(speeds);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
