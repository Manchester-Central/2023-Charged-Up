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
    public static final double MaxRobotSpeed_mps = 4.6; 
    public static final double MaxRobotRotation_radps = 5; 
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
    public static final double AbsoluteAngleOffsetFrontLeft = 238.3 - 180;
    public static final double AbsoluteAngleOffsetFrontRight =  231.6 - 180;
    public static final double AbsoluteAngleOffsetBackLeft = 327.2 - 180;
    public static final double AbsoluteAngleOffsetBackRight = 100.0 + 180;
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
      public static final int AbsoluteEncoderDIOPort = 0;
      public static final double AbsoluteAngleConversionFactor = 360; 
      public static final double AbsoluteAngleZeroOffset = -190; 
      public static final double SparkMaxEncoderConversionFactor = 2.6768; 
      public static final double MinimumAngleDegrees = -205; 
      public static final double MaximumAngleDegrees = 25; 
      public static final double MinDangerAngle = -120; //TODO confirm values
      public static final double MaxDangerAngle = -60; //TODO confirm values 
      public static final double RampUpRate = 0.15; //TODO confirm values
      public static final double ToleranceDegrees = 0.5; //TODO confirm values
      public static final double Gearing = 24576/180.0; //TODO confirm values
      public static final double MinimumFeedForwardVoltage = 0.45; // +0.02, -0.03 when measured on the robot in duty cycle (-1.0, 1.0)
      public static final double MaximumFeedForwardVoltage = 0.75;
      public static final double MaxPIDOutput = 1.0; //TODO confirm values
    }

    public static class ExtenderConstants{
      public static final int CanIdExtender = 14;
      public static final double LinearPotConversionFactor = 0.4286;
      public static final double LinearPotOffsetMeters = 0.68; 
      public static final double SparkMaxEncoderConversionFactor = 0.0069; 
      public static final double MinimumPositionMeters = 0.78 + 0.02; 
      public static final double MaximumPositionMeters = 1.32 - 0.02; 
      public static final double ExtenderSafeLimit = MinimumPositionMeters + 0.02; //TODO confirm values
      public static final double RampUpRate = 0.012; //TODO confirm values
      public static final double ToleranceMeters = 0.02; //TODO confirm values
      public static final double MaxPIDOutput = 1.0; //TODO confirm values
    }

    public static class WristConstants{
      public static final int CanIdWrist = 15;
      public static final double MinimumAngle = -10; //TODO confirm values
      public static final double MaximumAngle = 370; //TODO confirm values
      public static final double AbsoluteAngleConversionFactor = 462.8599853515625;
      public static final double AbsoluteAngleZeroOffset = 213;
      public static final double SparkMaxEncoderConversionFactor = 7.93;
      public static final double RampUpRate = 0.2; //TODO confirm values
      public static final double ToleranceDegrees = 0.5; //TODO confirm values
      public static final double MinimumSafeAngleDegrees = 150;
      public static final double MaximumSafeAngleDegrees = 210;
      public static final double MaxPIDOutput = 0.3;
    }

    public static class GripperConstants{
      public static final int CanIdGripper = 16;
    }
  } 

  public static class CommConstants {
    public static final String arduinoPort = "/dev/ttyUSB0"; // We need to determine what port the arduino will be connected to.
  }
}
