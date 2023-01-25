// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.Pigeon2;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }
  public static class SwerveConstants {
    public static final double RobotWidth_in = 21.25;
    public static final double RobotLength_in = 22.25;
    public static final double RobotWidth_m = RobotWidth_in / 39.3701;
    public static final double RobotLength_m = RobotLength_in / 39.3701;
    public static final double MaxRobotSpeed_mps = 5;
    public static final double MaxRobotRotation_radps = 2;
    public static final int CanIdFrontLeftAngle = 5; //TODO: 2022 value
    public static final int CanIdFrontLeftVelocity = 6; //TODO: 2022 value
    public static final int CanIdFrontRightAngle = 4; //TODO: 2022 value
    public static final int CanIdFrontRightVelocity = 8; //TODO: 2022 value
    public static final int CanIdBackLeftAngle = 7; //TODO: 2022 value
    public static final int CanIdBackLeftVelocity = 3; //TODO: 2022 value
    public static final int CanIdBackRightAngle = 1; //TODO: 2022 value
    public static final int CanIdBackRightVelocity = 2; //TODO: 2022 value
    public static final double AngleEncoderRatio = 144.0/14.0; //TODO: 2022 value
    public static final double VelocityEncoderRatio = 7.8; //TODO: 2022 value
    public static final double WheelDiameter = 0.092;
    public static final double WheelCircumference = WheelDiameter * Math.PI;
  }
   public static final double UpdateFrequency_Hz = 50;
   public static final boolean Is2022Robot = true;

}
