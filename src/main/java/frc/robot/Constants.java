// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
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
    public static final double MaxRobotSpeed_mps = 2.5; //TODO 3.8 is measured MAX
    public static final double MaxRobotRotation_radps = 1; //TODO revert rotation speed to 2 after testing.
    public static final int CanIdFrontLeftAngle = 2;
    public static final int CanIdFrontLeftVelocity = 1;
    public static final int CanIdFrontRightAngle = 4;
    public static final int CanIdFrontRightVelocity = 3;
    public static final int CanIdBackLeftAngle = 6;
    public static final int CanIdBackLeftVelocity = 5;
    public static final int CanIdBackRightAngle = 8;
    public static final int CanIdBackRightVelocity = 7;
    public static final double AngleEncoderRatio = 144.0 / 14.0; // TODO confirm
    public static final double VelocityEncoderRatio = 7.8; // TODO confirm
    public static final double WheelDiameter = 0.092;
    public static final double WheelCircumference = WheelDiameter * Math.PI;
    public static final int AnalogInputFrontLeftAbsoluteEncoder = 1;
    public static final int AnalogInputFrontRightAbsoluteEncoder = 2;
    public static final int AnalogInputBackLeftAbsoluteEncoder = 3;
    public static final int AnalogInputBackRightAbsoluteEncoder = 0;
    public static final double AbsoluteAngleOffsetFrontLeft = 244;
    public static final double AbsoluteAngleOffsetFrontRight =  164.2;
    public static final double AbsoluteAngleOffsetBackLeft = 213.5;
    public static final double AbsoluteAngleOffsetBackRight = 216.5;
  }

  public static class SwerveConstants2022 {
    public static final double RobotWidth_in = 21.25;
    public static final double RobotLength_in = 22.25;
    public static final double RobotWidth_m = RobotWidth_in / 39.3701;
    public static final double RobotLength_m = RobotLength_in / 39.3701;
    public static final double MaxRobotSpeed_mps = 5;
    public static final double MaxRobotRotation_radps = 2;
    public static final int CanIdFrontLeftAngle = 5;
    public static final int CanIdFrontLeftVelocity = 6;
    public static final int CanIdFrontLeftAbsoluteEncoder = 22;
    public static final int CanIdFrontRightAngle = 4;
    public static final int CanIdFrontRightVelocity = 8;
    public static final int CanIdFrontRightAbsoluteEncoder = 21;
    public static final int CanIdBackLeftAngle = 7;
    public static final int CanIdBackLeftVelocity = 3;
    public static final int CanIdBackLeftAbsoluteEncoder = 23;
    public static final int CanIdBackRightAngle = 1;
    public static final int CanIdBackRightVelocity = 2;
    public static final int CanIdBackRightAbsoluteEncoder = 20;
    public static final double AngleEncoderRatio = 144.0 / 14.0;
    public static final double VelocityEncoderRatio = 7.8;
    public static final double WheelDiameter = 0.092;
    public static final double WheelCircumference = WheelDiameter * Math.PI;
    public static final double AbsoluteAngleOffsetFrontLeft = 250;
    public static final double AbsoluteAngleOffsetFrontRight = 177;
    public static final double AbsoluteAngleOffsetBackLeft = 237;
    public static final double AbsoluteAngleOffsetBackRight = 341;
  }

  public static class FieldConstants {
    public static final double FieldWidth_m = 8.02;
    public static final double FieldLength_m = 16.54;
  }

  public static final double UpdateFrequency_Hz = 50;
  public static final boolean Is2022Robot = false;
  public static final double DriveToTargetTolerance = 0.03;
  public static final double AnglePIDTolerance = 3.0;

  public static class ArmConstants{
    public static class ShoulderConstants{
      public static final int CanIdShoulderL_A = 10;
      public static final int CanIdShoulderL_B = 11;
      public static final int CanIdShoulderR_A = 12;
      public static final int CanIdShoulderR_B = 13;
      public static final double AbsoluteAngleConversionFactor = 360; //TODO confirm values
      public static final double AbsoluteAngleZeroOffset = 0; //TODO confirm values
      public static final double SparkMaxEncoderConversionFactor = 1; //TODO confirm values
      public static final double MinimumAngle = -225; //TODO confirm values
      public static final double MaximumAngle = 45; //TODO confirm values
      public static final double MinDangerAngle = -120; //TODO confirm values
      public static final double MaxDangerAngle = -60; //TODO confirm values 
    }

    public static class ExtenderConstants{
      public static final int CanIdExtender = 14;
      public static final double LinearPotConversionFactor = 1; //TODO confirm values
      public static final double LinearPotOffsetMeters = 0; //TODO confirm values
      public static final double SparkMaxEncoderConversionFactor = 1; //TODO confirm values
      public static final double MinimumPositionMeters = 0.78; //TODO confirm values
      public static final double MaximumPositionMeters = 1.32; //TODO confirm values
      public static final double ExtenderSafeLimit = 1; //TODO confirm values
    }

    public static class WristConstants{
      public static final int CanIdWrist = 15;
      public static final double MinimumAngle = -90; //TODO confirm values
      public static final double MaximumAngle = 90; //TODO confirm values
      public static final double AbsoluteAngleConversionFactor = 360; //TODO confirm values
      public static final double AbsoluteAngleZeroOffset = 0; //TODO confirm values
      public static final double SparkMaxEncoderConversionFactor = 1; //TODO confirm values
    }
  } 
}
