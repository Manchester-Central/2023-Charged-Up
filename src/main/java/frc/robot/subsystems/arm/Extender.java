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

/** Add your docs here. */
public class Extender {
    private CANSparkMax m_SparkMax;
    private PIDTuner m_pidTuner;
    private SparkMaxAnalogSensor m_linearPot;
    private SafetyZoneHelper m_SafetyZoneHelper;
    private double m_simPos = ExtenderConstants.MinimumPositionMeters;
    private double m_targetMeters = m_simPos;
    public Extender(){
        m_SparkMax = new CANSparkMax(ExtenderConstants.CanIdExtender, MotorType.kBrushless);
        m_SparkMax.setInverted(true);
        m_SparkMax.setIdleMode(IdleMode.kBrake);
        m_pidTuner = new PIDTuner("ExtenderPID", true, 80, 0, 0, this::tunePID);
        m_linearPot = m_SparkMax.getAnalog(Mode.kAbsolute);
        m_linearPot.setPositionConversionFactor(ExtenderConstants.LinearPotConversionFactor);
        m_SparkMax.getEncoder().setPositionConversionFactor(ExtenderConstants.SparkMaxEncoderConversionFactor);
        recalibrateSensors();
        m_SafetyZoneHelper = new SafetyZoneHelper(ExtenderConstants.MinimumPositionMeters, ExtenderConstants.MaximumPositionMeters);
        m_SparkMax.setOpenLoopRampRate(ExtenderConstants.RampUpRate);
        m_SparkMax.setClosedLoopRampRate(ExtenderConstants.RampUpRate);
        m_SparkMax.getPIDController().setOutputRange(-ExtenderConstants.MaxPIDOutput, ExtenderConstants.MaxPIDOutput);
        m_SparkMax.burnFlash();
        Robot.logManager.addNumber("Extender/ExtensionMeters", () -> getPositionMeters());
        Robot.logManager.addNumber("Extender/SparkMaxMeters", () -> m_SparkMax.getEncoder().getPosition());
        Robot.logManager.addNumber("Extender/appliedOutput", () -> m_SparkMax.getAppliedOutput());
        SmartDashboard.putNumber("Extender/maxOutput", ExtenderConstants.MaxPIDOutput);
        SmartDashboard.putNumber("Extender/rampRate", ExtenderConstants.RampUpRate);
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
        m_SparkMax.getPIDController().setReference(safeTargetPosition, ControlType.kPosition);
        
    }

    public double getPositionMeters(){
        if (Robot.isSimulation()) {
            return m_simPos;
        }
        return m_linearPot.getPosition() + ExtenderConstants.LinearPotOffsetMeters;
    }

    public void tunePID(PIDUpdate pidUpdate){
        m_SparkMax.getPIDController().setP(pidUpdate.P);
        m_SparkMax.getPIDController().setI(pidUpdate.I);        
        m_SparkMax.getPIDController().setD(pidUpdate.D);
        m_SparkMax.getPIDController().setFF(pidUpdate.F);
        var maxOutput = SmartDashboard.getNumber("Extender/maxOutput", ExtenderConstants.MaxPIDOutput);
        var rampRate = SmartDashboard.getNumber("Extender/rampRate", ExtenderConstants.RampUpRate);
        m_SparkMax.getPIDController().setOutputRange(-maxOutput, maxOutput);
        m_SparkMax.setOpenLoopRampRate(rampRate);
        m_SparkMax.setClosedLoopRampRate(rampRate);
    }

    public boolean atTarget(){
        return Math.abs(getPositionMeters() - m_targetMeters) < ExtenderConstants.ToleranceMeters;
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
        m_SparkMax.set(speed);
    }
    
    public void stop(){
        //TODO After testing, should remain at current position instead.
        m_SparkMax.stopMotor();
        m_targetMeters = m_simPos;
    }

    public void recalibrateSensors() {
        m_SparkMax.getEncoder().setPosition(getPositionMeters());
    }
}

//“MMMMMMM, Mcextender.” -John, 2/20/2023