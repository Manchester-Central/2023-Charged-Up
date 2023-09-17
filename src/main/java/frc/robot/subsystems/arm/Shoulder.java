// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.chaos131.pid.PIDFValue;
import com.chaos131.pid.PIDTuner;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.ArmConstants.ExtenderConstants;
import frc.robot.Constants.ArmConstants.ShoulderConstants;
import frc.robot.Constants.DebugConstants;
import frc.robot.Robot;

/** Add your docs here. */
public class Shoulder {
    CANSparkMax m_shoulderL_A;
    CANSparkMax m_shoulderL_B;
    CANSparkMax m_shoulderR_A;
    CANSparkMax m_shoulderR_B;
    DutyCycleEncoder m_AbsoluteEncoder;
    PIDTuner m_pidTuner;
    SafetyZoneHelper m_SafetyZoneHelper;
    double m_targetDegrees = Double.NaN;
    PIDController m_simPid = new PIDController(0.025, 0, 0); //d in simulator was causing issues, so simPID will use it's own values

    private final double kSimExtenderFixedPosition = ExtenderConstants.MaximumPositionMeters;

    final SingleJointedArmSim m_armSim = 
    new SingleJointedArmSim(
        DCMotor.getNEO(4), 
        ShoulderConstants.Gearing, 
        SingleJointedArmSim.estimateMOI(kSimExtenderFixedPosition, 13.687), 
        kSimExtenderFixedPosition, 
        Rotation2d.fromDegrees(ShoulderConstants.MinimumAngleDegrees).getRadians(), 
        Rotation2d.fromDegrees(ShoulderConstants.MaximumAngleDegrees).getRadians(), 
        true
        );

    public Shoulder () {
        m_shoulderL_A = new CANSparkMax(ShoulderConstants.CanIdShoulderL_A, MotorType.kBrushless);
        m_shoulderL_B = new CANSparkMax(ShoulderConstants.CanIdShoulderL_B, MotorType.kBrushless);
        m_shoulderR_A = new CANSparkMax(ShoulderConstants.CanIdShoulderR_A, MotorType.kBrushless);
        m_shoulderR_B = new CANSparkMax(ShoulderConstants.CanIdShoulderR_B, MotorType.kBrushless);
        m_shoulderL_A.getPIDController().setFF(0);
        m_shoulderL_B.getPIDController().setFF(0);
        m_shoulderR_A.getPIDController().setFF(0);
        m_shoulderR_B.getPIDController().setFF(0);
        m_shoulderL_A.setInverted(true);
        m_shoulderL_B.setInverted(true);
        m_shoulderR_A.setInverted(false);
        m_shoulderR_B.setInverted(false);
        m_AbsoluteEncoder = new DutyCycleEncoder(ShoulderConstants.AbsoluteEncoderDIOPort);
        // m_AbsoluteEncoder.setDistancePerRotation(ShoulderConstants.AbsoluteAngleConversionFactor);
        // m_AbsoluteEncoder.setPositionOffset(ShoulderConstants.AbsoluteAngleZeroOffset);
        CANSparkMax[] motorControllers = {m_shoulderL_A, m_shoulderL_B, m_shoulderR_A, m_shoulderR_B};
        for (CANSparkMax canSparkMax : motorControllers) {
            canSparkMax.setIdleMode(IdleMode.kBrake);
            initializeSparkMaxEncoder(canSparkMax, getRotation());
            canSparkMax.setOpenLoopRampRate(ShoulderConstants.RampUpRate);
            canSparkMax.setClosedLoopRampRate(ShoulderConstants.RampUpRate);
            canSparkMax.getPIDController().setOutputRange(-ShoulderConstants.MaxPIDOutput, ShoulderConstants.MaxPIDOutput);
            //canSparkMax.setSmartCurrentLimit(15, 20, 8000);
            canSparkMax.setSmartCurrentLimit(0, 0, 0);
            //open loop = no pid, closed loop = pid
            canSparkMax.burnFlash();
            Robot.logManager.addNumber("Shoulder/SparkMax" + canSparkMax.getDeviceId() + "/MotorTemperature_C", DebugConstants.EnableArmDebug, () -> canSparkMax.getMotorTemperature());

        }
        m_pidTuner = new PIDTuner("Shoulder/PID_Tuner", DebugConstants.EnableArmDebug, 0.025, 0, 1.6, this::tunePID);
        m_SafetyZoneHelper = new SafetyZoneHelper(ShoulderConstants.MinimumAngleDegrees, ShoulderConstants.MaximumAngleDegrees);
        Robot.logManager.addNumber("Shoulder/target", DebugConstants.EnableArmDebug, () -> m_targetDegrees);
        Robot.logManager.addNumber("Shoulder/Rotation_deg", DebugConstants.EnableArmDebug, () -> getRotation().getDegrees());
        Robot.logManager.addNumber("Shoulder/AngleFromStowed_deg", DebugConstants.EnableArmDebug, () -> getShoulderDegreesFromStowed());
    }
    
    public void addCoachTabDashboardValues(ShuffleboardTab coachTab) {
        
    }

    private Rotation2d getRotation() {
        if(Robot.isSimulation()) {
            return Rotation2d.fromRadians(m_armSim.getAngleRads());
        }
        return Rotation2d.fromDegrees((m_AbsoluteEncoder.getAbsolutePosition() * ShoulderConstants.AbsoluteAngleConversionFactor) + ShoulderConstants.AbsoluteAngleZeroOffset);
    }

    public double getShoulderDegreesFromStowed() {
      return Math.abs(getEncoderRotation().plus(Rotation2d.fromDegrees(90)).getDegrees()); 
    }

    public Rotation2d getEncoderRotation() {
        if(Robot.isSimulation()) {
            return Rotation2d.fromRadians(m_armSim.getAngleRads());
        }
        return Rotation2d.fromDegrees(m_shoulderL_A.getEncoder().getPosition());
    }

    public void updateSafetyZones(ArmPose targetArmPose, double extenderLengthMeters, Rotation2d wristAngle) {
        if (extenderLengthMeters >= ExtenderConstants.ExtenderSafeLimit) {
            double normalizedCurrentAngle = normalize(getEncoderRotation());
            if (normalizedCurrentAngle < -90) {
                m_SafetyZoneHelper.excludeUp(ShoulderConstants.MinDangerAngle);
            } else{
                m_SafetyZoneHelper.excludeDown(ShoulderConstants.MaxDangerAngle);
            }
        }
        else {
            m_SafetyZoneHelper.resetToDefault();
        }
    }

    public void setTargetAngle(Rotation2d targetAngle, double currentExtensionMeters) {
        double targetDegrees = normalize(targetAngle);
        double feedForwardVoltage = getArbitraryFeedForward(currentExtensionMeters);
        targetDegrees = m_SafetyZoneHelper.getSafeValue(targetDegrees);
        m_targetDegrees = targetDegrees;
        m_shoulderL_A.getPIDController().setReference(targetDegrees, ControlType.kPosition, 0, feedForwardVoltage);
        m_shoulderL_B.getPIDController().setReference(targetDegrees, ControlType.kPosition, 0, feedForwardVoltage);
        m_shoulderR_A.getPIDController().setReference(targetDegrees, ControlType.kPosition, 0, feedForwardVoltage);
        m_shoulderR_B.getPIDController().setReference(targetDegrees, ControlType.kPosition, 0, feedForwardVoltage);
    }

    public static double normalize(double targetDegrees) {
        targetDegrees %= 360;
        if (targetDegrees <= -269) {
            targetDegrees += 360;
        } else if (targetDegrees >= 89) {
            targetDegrees -= 360;
        }
        return targetDegrees;
    } 
    // we have positions, degrees are associated with them

    public static double normalize(Rotation2d targetRotation2d) {
        return normalize(targetRotation2d.getDegrees());
    }


    public void tunePID(PIDFValue pidUpdate) {
        setPID(pidUpdate, m_shoulderL_A);
        setPID(pidUpdate, m_shoulderL_B);
        setPID(pidUpdate, m_shoulderR_A);
        setPID(pidUpdate, m_shoulderR_B);
    }

    public void setPID(PIDFValue pidUpdate, CANSparkMax sparkMax) {
        var controller = sparkMax.getPIDController();
        controller.setP(pidUpdate.P);
        controller.setI(pidUpdate.I);
        controller.setD(pidUpdate.D);
    }
    
    private void initializeSparkMaxEncoder(CANSparkMax sparkMax, Rotation2d absoluteAngle) {
       RelativeEncoder encoder = sparkMax.getEncoder();
       encoder.setPositionConversionFactor(ShoulderConstants.SparkMaxEncoderConversionFactor);
       encoder.setPosition(normalize(absoluteAngle));
       Robot.logManager.addNumber("Shoulder/SparkMax" + sparkMax.getDeviceId() + "/TranslatedAngle", DebugConstants.EnableArmDebug, () -> encoder.getPosition());
    }

    public boolean atTarget(double targetDegrees){
        if (Double.isNaN(targetDegrees)) {
            return false;
        }
        var dist = Math.abs(getEncoderRotation().getDegrees() - targetDegrees);
        // SmartDashboard.putNumber("shoulderDist", dist);
        return dist < ShoulderConstants.ToleranceDegrees;
    }

    // We want to add an arbitrary feed forward that applies outside the PID control loop.
    // https://docs.revrobotics.com/sparkmax/operating-modes/closed-loop-control
    private double getArbitraryFeedForward(double currentExtensionMeters) {
        double slope = (ShoulderConstants.MaximumFeedForwardVoltage - ShoulderConstants.MinimumFeedForwardVoltage) / 
        (ExtenderConstants.MaximumPositionMeters - ExtenderConstants.MinimumPositionMeters);
        double offset = ShoulderConstants.MinimumFeedForwardVoltage - (ExtenderConstants.MinimumPositionMeters * slope);
        double maxFeedForward = (slope * currentExtensionMeters) + offset;
        return maxFeedForward * getEncoderRotation().getCos();
    }

    public void periodic(double currentExtensionMeters) {
        m_pidTuner.tune();
        if (Robot.isSimulation()) {
            double feedForwardVoltage = getArbitraryFeedForward(kSimExtenderFixedPosition);
            if(DriverStation.isEnabled() && Double.isFinite(m_targetDegrees)) {
                double voltage = MathUtil.clamp(m_simPid.calculate(getRotation().getDegrees(), m_targetDegrees), -1, 1) * RobotController.getBatteryVoltage();
                double voltageWithFeedForward = voltage + feedForwardVoltage;
                m_armSim.setInput(voltageWithFeedForward);
            } else{
                m_armSim.setInput(0);
            }
            m_armSim.update(0.02);          
        }
    }

    public void setManual(double speed) {
        m_shoulderL_A.set(speed);
        m_shoulderL_B.set(speed);
        m_shoulderR_A.set(speed);
        m_shoulderR_B.set(speed);
    }

    public void stop() {
     //TODO After testing, should remain at current position instead.
        m_shoulderL_A.stopMotor();
        m_shoulderL_B.stopMotor();
        m_shoulderR_A.stopMotor();
        m_shoulderR_B.stopMotor();
        m_targetDegrees = Double.NaN;
    }

    public void recalibrateSensors() {
        m_shoulderL_A.getEncoder().setPosition(normalize(getRotation()));
        m_shoulderL_B.getEncoder().setPosition(normalize(getRotation()));
        m_shoulderR_A.getEncoder().setPosition(normalize(getRotation()));
        m_shoulderR_B.getEncoder().setPosition(normalize(getRotation()));
    }
}
