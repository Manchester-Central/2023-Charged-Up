// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.chaos131.pid.PIDTuner;
import com.chaos131.pid.PIDUpdate;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAnalogSensor.Mode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants.ArmConstants.ExtenderConstants;
import frc.robot.Constants.ArmConstants.ShoulderConstants;
import frc.robot.util.DashboardNumber;

/** Add your docs here. */
public class Extender {
    private CANSparkMax m_sparkMax;
    private PIDTuner m_pidTuner;
    private SparkMaxAnalogSensor m_linearPot;
    private SafetyZoneHelper m_SafetyZoneHelper;
    private double m_simPos = ExtenderConstants.MinimumPositionMeters;
    private double m_targetMeters = m_simPos;
    public Extender(){
        m_sparkMax = new CANSparkMax(ExtenderConstants.CanIdExtender, MotorType.kBrushless);
        m_sparkMax.setInverted(true);
        m_sparkMax.setIdleMode(IdleMode.kBrake);
        m_pidTuner = new PIDTuner("ExtenderPID", false, 80, 0, 0, this::tunePID);
        m_linearPot = m_sparkMax.getAnalog(Mode.kAbsolute);
        m_linearPot.setPositionConversionFactor(ExtenderConstants.LinearPotConversionFactor);
        new DashboardNumber("Extender/EncoderConversionFactor", ExtenderConstants.SparkMaxEncoderConversionFactor, (newConversionFactor) -> {
            m_sparkMax.getEncoder().setPositionConversionFactor(newConversionFactor);
            recalibrateSensors();
        });
        m_SafetyZoneHelper = new SafetyZoneHelper(ExtenderConstants.MinimumPositionMeters, ExtenderConstants.MaximumPositionMeters);
        m_sparkMax.setOpenLoopRampRate(ExtenderConstants.RampUpRate);
        m_sparkMax.setClosedLoopRampRate(ExtenderConstants.RampUpRate);
        m_sparkMax.getPIDController().setOutputRange(-ExtenderConstants.MaxPIDOutput, ExtenderConstants.MaxPIDOutput);
        //m_sparkMax.setSmartCurrentLimit(15, 20, 8000);
        m_sparkMax.setSmartCurrentLimit(0, 0, 0);
        m_sparkMax.burnFlash();
        Robot.logManager.addNumber("Extender/SparkMaxMeters", () -> m_sparkMax.getEncoder().getPosition());
    }

    public void updateSafetyZones(ArmPose targetArmPose, Rotation2d shoulderAngle){
        double normalizedCurrentAngle = Shoulder.normalize(shoulderAngle);
        double normalizedTargetAngle = Shoulder.normalize(targetArmPose.shoulderAngle);
        if ((normalizedCurrentAngle < ShoulderConstants.MinDangerAngle && normalizedTargetAngle < ShoulderConstants.MinDangerAngle)
        || (normalizedCurrentAngle > ShoulderConstants.MaxDangerAngle && normalizedTargetAngle > ShoulderConstants.MaxDangerAngle)) {
            m_SafetyZoneHelper.resetToDefault();
        } else{
            m_SafetyZoneHelper.excludeUp(ExtenderConstants.MinimumPositionMeters);
        }
    }

    public void ExtendToTarget(double targetPositionMeters) {

        double safeTargetPosition = m_SafetyZoneHelper.getSafeValue(targetPositionMeters);

        m_targetMeters = safeTargetPosition;
        m_sparkMax.getPIDController().setReference(safeTargetPosition, ControlType.kPosition);
        
    }

    public double getPositionMeters(){
        if (Robot.isSimulation()) {
            return m_simPos;
        }
        return m_linearPot.getPosition() + ExtenderConstants.LinearPotOffsetMeters;
    }

    public void tunePID(PIDUpdate pidUpdate){
        m_sparkMax.getPIDController().setP(pidUpdate.P);
        m_sparkMax.getPIDController().setI(pidUpdate.I);        
        m_sparkMax.getPIDController().setD(pidUpdate.D);
        m_sparkMax.getPIDController().setFF(pidUpdate.F);
    }

    public boolean atTarget(){
        return Math.abs(m_sparkMax.getEncoder().getPosition() - m_targetMeters) < ExtenderConstants.ToleranceMeters;
    }

    public void periodic() {
        double increment = 0.012;
        if (Math.abs(m_simPos - m_targetMeters) <= Math.abs(increment)) {
            m_simPos = m_targetMeters;
        } else if(m_targetMeters <= m_simPos) {
            m_simPos -= increment;
        } else {
            m_simPos += increment; 
        }
        m_pidTuner.tune();
    }

    public void setManual(double speed) {
        m_sparkMax.set(speed);
    }
    
    public void stop(){
        //TODO After testing, should remain at current position instead.
        m_sparkMax.stopMotor();
        m_targetMeters = m_simPos;
    }

    public void recalibrateSensors() {
        m_sparkMax.getEncoder().setPosition(getPositionMeters());
    }
}

//“MMMMMMM, Mcextender.” -John, 2/20/2023