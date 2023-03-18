// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fazecast.jSerialComm.SerialPort;

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
    public static final int kTesterControllerPort = 2;
  }

  public static class SwerveConstants {
    public static final double RobotWidth_in = 21.25;
    public static final double RobotLength_in = 22.25;
    public static final double RobotWidth_m = RobotWidth_in / 39.3701;
    public static final double RobotLength_m = RobotLength_in / 39.3701;
    public static final double MaxRobotSpeed_mps = 4.6; //4.6 
    public static final double MaxRobotRotation_radps = 3; //5
    public static final int CanIdFrontLeftAngle = 1;
    public static final int CanIdFrontLeftVelocity = 2;
    public static final int CanIdFrontRightAngle = 4;
    public static final int CanIdFrontRightVelocity = 3;
    public static final int CanIdBackLeftAngle = 6;
    public static final int CanIdBackLeftVelocity = 5;
    public static final int CanIdBackRightAngle = 8;
    public static final int CanIdBackRightVelocity = 7;
    public static final double AngleEncoderRatio = 144.0 / 14.0;
    public static final double VelocityEncoderRatio = 72 / 11;
    public static final double WheelDiameter = 0.092;
    public static final double WheelCircumference = WheelDiameter * Math.PI;
    public static final int AnalogInputFrontLeftAbsoluteEncoder = 3;
    public static final int AnalogInputFrontRightAbsoluteEncoder = 0;
    public static final int AnalogInputBackLeftAbsoluteEncoder = 2;
    public static final int AnalogInputBackRightAbsoluteEncoder = 1;
    public static final double AbsoluteAngleOffsetFrontLeft = 17.4;
    public static final double AbsoluteAngleOffsetFrontRight =  191.9;
    public static final double AbsoluteAngleOffsetBackLeft = 286.7;
    public static final double AbsoluteAngleOffsetBackRight = 60.2;
    public static final double SlewRateLimit = 0.2;
  }

  public static class FieldConstants {
    public static final double FieldWidth_m = 8.02;
    public static final double FieldLength_m = 16.54;
  }

  public static final double UpdateFrequency_Hz = 50;
  public static final double DriveToTargetTolerance = 0.03;
  public static final double AnglePIDTolerance = 3.0;

  public static class ArmConstants{
    public static class ShoulderConstants{
      public static final int CanIdShoulderL_A = 10;
      public static final int CanIdShoulderL_B = 11;
      public static final int CanIdShoulderR_A = 12;
      public static final int CanIdShoulderR_B = 13;
      public static final int AbsoluteEncoderDIOPort = 0;
      public static final double AbsoluteAngleConversionFactor = 360; 
      public static final double AbsoluteAngleZeroOffset = -190; 
      public static final double SparkMaxEncoderConversionFactor = 2.6768; 
      public static final double MinimumAngleDegrees = -205; 
      public static final double MaximumAngleDegrees = 25; 
      public static final double MinDangerAngle = -110; 
      public static final double MaxDangerAngle = -70; 
      public static final double RampUpRate = 0.2; 
      public static final double ToleranceDegrees = 1.5; 
      public static final double Gearing = 24576/180.0; 
      public static final double MinimumFeedForwardVoltage = 0.45; // +0.02, -0.03 when measured on the robot in duty cycle (-1.0, 1.0)
      public static final double MaximumFeedForwardVoltage = 0.75;
      public static final double MaxPIDOutput = 1.0; 
    }

    public static class ExtenderConstants{
      public static final int CanIdExtender = 14;
      public static final double LinearPotConversionFactor = 0.4286;
      public static final double LinearPotOffsetMeters = 0.68; 
      public static final double SparkMaxEncoderConversionFactor = 0.0044538; 
      public static final double MinimumPositionMeters = 0.78 + 0.02; 
      public static final double MaximumPositionMeters = 1.32 - 0.02; 
      public static final double ExtenderSafeLimit = MinimumPositionMeters + 0.03; 
      public static final double RampUpRate = 0.012;
      public static final double ToleranceMeters = 0.02; 
      public static final double MaxPIDOutput = 1.0; 
    }

    public static class WristConstants{
      public static final int CanIdWrist = 15;
      public static final double MinimumAngle = -30; 
      public static final double MaximumAngle = 410; 
      public static final double AbsoluteAngleConversionFactor = 462.8599853515625;
      public static final double AbsoluteAngleZeroOffset = 130.0;
      public static final double SparkMaxEncoderConversionFactor = 5.803;
      public static final double RampUpRate = 0.1; 
      public static final double ToleranceDegrees = 2.0; 
      public static final double MinimumSafeAngleDegrees = 150;
      public static final double MaximumSafeAngleDegrees = 210;
      public static final double MaxPIDOutput = 1.0;
    }

    public static class GripperConstants{
      public static final int CanIdGripper = 16;
    }
  } 

  public static class CommConstants {
    public static final String arduinoPort = "/dev/ttyS0"; // We need to determine what port the arduino will be connected to.
  }
}
