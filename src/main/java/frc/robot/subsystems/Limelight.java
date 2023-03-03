// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
  String m_tableName;

  /** Creates a new Limelight. */
  public Limelight(String tableName) {
    m_tableName = tableName;
    m_limelightTable = NetworkTableInstance.getDefault().getTable(m_tableName);
    m_field = new Field2d();
    SmartDashboard.putData(m_tableName + "/CameraPosition", m_field);
  }

  public Pose2d getPose() {
    double defaults[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    double[] results = m_limelightTable.getEntry("botpose").getDoubleArray(defaults);
    double limeLightOriginX = FieldConstants.FieldLength_m / 2;
    double limeLightOriginY = FieldConstants.FieldWidth_m / 2;

    if (results.length >= 5) {
      double x = results[0] + limeLightOriginX;
      double y = results[1] + limeLightOriginY;
      Pose2d pose = new Pose2d(x, y, Rotation2d.fromDegrees(results[5]));
      return pose;
    } else {
      return new Pose2d();
    }
  }

  public int getPipeline() {
    return (int) m_limelightTable.getEntry("getpipe").getDouble(-1);
  }

  public void setPipeline(int pipeline) {
    m_limelightTable.getEntry("pipeline").setDouble(pipeline);
  }

  public boolean hasTarget() {
    return m_limelightTable.getEntry("tv").getDouble(0) == 1.0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.printf("TargetCheck %b \n", hasTarget());
    m_field.setRobotPose(getPose());
    SmartDashboard.putNumber(m_tableName + "/pipeline", getPipeline());
    
  }
}
