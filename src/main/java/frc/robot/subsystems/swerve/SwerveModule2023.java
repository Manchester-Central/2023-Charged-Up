// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.chaos131.pid.PIDFValue;
import com.chaos131.swerve.BaseSwerveModule;
import com.chaos131.util.DashboardNumber;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DebugConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Robot;

/** Add your docs here. */
public class SwerveModule2023 extends BaseSwerveModule {

    private AnalogEncoder m_absoluteEncoder;
    private double m_absoluteAngleOffset;

    private WPI_TalonFX m_angle;
    private WPI_TalonFX m_velocity;
  
    private static DashboardNumber VelocityRampRateDriver = new DashboardNumber("Swerve/VelocityRampRateDriver", 0.2, DebugConstants.EnableDriveDebug, (newValue) -> {});
    private static DashboardNumber VelocityRampRateAuto = new DashboardNumber("Swerve/VelocityRampRateAuto", 0.5, DebugConstants.EnableDriveDebug, (newValue) -> {});
  
    /** Creates a new SwerveModule2023. */
    public SwerveModule2023(String moduleName, Translation2d translation, int canIdAngle, int canIdVelocity, int absoluteAnalogPort, double absoluteAngleOffset, Rotation2d xModeAngle) {
        super(moduleName, translation, xModeAngle);

        m_absoluteEncoder = new AnalogEncoder(absoluteAnalogPort);

        new DashboardNumber(getDSKey("absoluteAngleOffset"), absoluteAngleOffset, DebugConstants.EnableDriveDebug,  (newValue) -> {
            m_absoluteAngleOffset = newValue;
            recalibrate();
        });
        m_angle = new WPI_TalonFX(canIdAngle);
        m_velocity = new WPI_TalonFX(canIdVelocity);
        m_angle.configFactoryDefault();
        m_velocity.configFactoryDefault();
        m_angle.configAllowableClosedloopError(0, degreesToEncoder(0.5));
        m_velocity.setNeutralMode(NeutralMode.Coast);
        m_angle.setNeutralMode(NeutralMode.Brake);
        m_velocity.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_20Ms);
        m_velocity.configVelocityMeasurementWindow(32);
        m_angle.setInverted(TalonFXInvertType.Clockwise);
    
        m_velocity.configClosedloopRamp(VelocityRampRateDriver.get());
        m_angle.configClosedloopRamp(0);
    
        Robot.logManager.addNumber(getDSKey("Velocity_Temp_C"), DebugConstants.EnableDriveDebug, () -> m_velocity.getTemperature());
        Robot.logManager.addNumber(getDSKey("Angle_Temp_C"), DebugConstants.EnableDriveDebug, () -> m_angle.getTemperature());
  
    }
      
    public void addCoachTabDashboardValues(ShuffleboardTab coachTab) {
      
    }
  
    public void driverModeInit() {
      m_velocity.configClosedloopRamp(VelocityRampRateDriver.get());
    }
  
    public void driveToPositionInit() {
      m_velocity.configClosedloopRamp(VelocityRampRateAuto.get());
    }
  
    public void UpdateVelocityPIDConstants(PIDFValue update) {
      m_velocity.config_kP(0, update.P);
      m_velocity.config_kI(0, update.I);
      m_velocity.config_kD(0, update.D);
      m_velocity.config_kF(0, update.F);
    }
  
    public void UpdateAnglePIDConstants(PIDFValue update) {
      m_angle.config_kP(0, update.P);
      m_angle.config_kI(0, update.I);
      m_angle.config_kD(0, update.D);
    }
  
    protected void updateDashboard() {
      if (DebugConstants.EnableDriveDebug) {
        SmartDashboard.putNumber(getDSKey("Angle"), getModuleState().angle.getDegrees());
        SmartDashboard.putNumber(getDSKey("Speed"), getModuleState().speedMetersPerSecond);
        SmartDashboard.putNumber(getDSKey("TargetSpeed"), m_targetState.speedMetersPerSecond);
        SmartDashboard.putNumber(getDSKey("Current_amps"), m_velocity.getStatorCurrent());
        // SmartDashboard.putNumber(getDSKey("VelocityEncoderPosition"), m_velocity.getSelectedSensorPosition());
        // SmartDashboard.putNumber(getDSKey("AngleEncoder"), m_angle.getSelectedSensorPosition());
        // SmartDashboard.putNumber(getDSKey("Position"), getPosition().distanceMeters);
        // SmartDashboard.putNumber(getDSKey("VelocityEncoderVelocity"), m_velocity.getSelectedSensorVelocity());
        SmartDashboard.putNumber(getDSKey("AbsoluteAngle"), getAbsoluteAngle().getDegrees());
        SmartDashboard.putNumber(getDSKey("AbsoluteAngleRollingAverage"), m_absoluteAngleDegreesRollingAverageValue);
        // SmartDashboard.putNumber(getDSKey("InitialEncoder"), initialEncoder);
        // SmartDashboard.putNumber(getDSKey("AbsoluteEncoder"), getRawAbsoluteAngle());
      }
    }
  
    public double encoderToDegrees(double counts) {
      counts = counts / 2048;
      counts = counts / getAngleEncoderRatio();
      return counts * 360;
    }
  
    public double degreesToEncoder(double angle) {
      angle = angle * 2048;
      angle = angle * getAngleEncoderRatio();
      return angle / 360;
  
    }
  
    public double encoderToDistanceMeters(double counts) {
      counts = counts / 2048;
      counts = counts / getVelocityEncoderRatio();
      return counts * getWheelCircumference();
  
    }
  
    public double distanceMetersToEncoders(double distance) {
      distance = distance * 2048;
      distance = distance * getVelocityEncoderRatio();
      return distance / getWheelCircumference();
  
    }
  
    public double velocityUnitToMeterPerSecond(double velocityUnit){
      return encoderToDistanceMeters(velocityUnit) * 10;
    }
  
    public double meterPerSecondToVelocityUnit(double metersPerSecond){
      return distanceMetersToEncoders(metersPerSecond) / 10;
    }

    public double getRawAbsoluteAngle(){
        return (m_absoluteEncoder.getAbsolutePosition() * 360);
    }

    public Rotation2d getAbsoluteAngle() {
        return Rotation2d.fromDegrees((m_absoluteEncoder.getAbsolutePosition() * 360) - m_absoluteAngleOffset);
    }

    public double getAngleEncoderRatio() {
        return SwerveConstants.AngleEncoderRatio;
    }

    public double getVelocityEncoderRatio() {
        return SwerveConstants.VelocityEncoderRatio;
    }

    public double getWheelCircumference() {
        return SwerveConstants.WheelCircumference;
    }

    @Override
    protected Rotation2d getEncoderAngle() {
        return Rotation2d.fromDegrees(encoderToDegrees(m_angle.getSelectedSensorPosition()));
    }

    @Override
    protected double getEncoderDistance_m() {
        return encoderToDistanceMeters(m_velocity.getSelectedSensorPosition());
    }

    @Override
    protected double getEncoderVelocity_mps() {
        return velocityUnitToMeterPerSecond(m_velocity.getSelectedSensorVelocity());
    }

    @Override
    protected void recalibrateWithFilteredAbsoluteAngle(Rotation2d filteredAngle) {
        var initialEncoder = degreesToEncoder(filteredAngle.getDegrees());
        m_angle.setSelectedSensorPosition(initialEncoder);
    }

    @Override
    protected void setTargetAngle(Rotation2d angle) {
        m_angle.set(TalonFXControlMode.Position, degreesToEncoder(angle.getDegrees()));
    }

    @Override
    protected void setTargetVelocity_mps(double velocity_mps) {
        m_velocity.set(TalonFXControlMode.Velocity, meterPerSecondToVelocityUnit(velocity_mps));
    }

    @Override
    protected void stopAngleMotor() {
        m_angle.set(TalonFXControlMode.PercentOutput, 0);
    }

    @Override
    protected void stopVelocityMotor() {
        m_velocity.set(TalonFXControlMode.PercentOutput, 0);
    }
}
