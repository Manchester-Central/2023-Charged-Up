// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;

public class Limelight extends SubsystemBase {
   NetworkTable m_limelightTable;
  Field2d m_field;
  /** Creates a new Limelight. */
  public Limelight() {
    m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    m_field = new Field2d();
    SmartDashboard.putData("CameraPosition", m_field);
  }

public Pose2d getPose() {
  double defaults[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double[] results = m_limelightTable.getEntry("botpose").getDoubleArray(defaults);
  double limeLightOriginX = FieldConstants.FieldLength_m/2;
  double limeLightOriginY= FieldConstants.FieldWidth_m/2;
  double x = results[0] + limeLightOriginX;
  double y = results[1] + limeLightOriginY;
  Pose2d pose = new Pose2d(x, y, Rotation2d.fromDegrees(results[5]));
  
  return pose;
}

public boolean hasTarget() {
  return m_limelightTable.getEntry("tv").getDouble(0) == 1.0;
}

  @Override 
  public void periodic() {
    // This method will be called once per scheduler run
    //System.out.printf("TargetCheck %b \n", hasTarget());
    m_field.setRobotPose(getPose());
  }
}
