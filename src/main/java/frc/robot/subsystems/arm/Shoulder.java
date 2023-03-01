// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.chaos131.pid.PIDTuner;
import com.chaos131.pid.PIDUpdate;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.ArmConstants.ExtenderConstants;
import frc.robot.Constants.ArmConstants.ShoulderConstants;
import frc.robot.Constants.ArmConstants.WristConstants;
import frc.robot.logging.LogManager;

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
    PIDController m_simPid = new PIDController(0, 0, 0);

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
            //open loop = no pid, closed loop = pid
        }
        m_pidTuner = new PIDTuner("ShoulderPID", true, 0.01, 0, 0, this::tunePID);
        Robot.logManager.addNumber("Shoulder/Shoulder_rotation", () -> getRotation().getDegrees());
        Robot.logManager.addNumber("Shoulder/appliedOutput", () -> m_shoulderL_A.getAppliedOutput());
        m_SafetyZoneHelper = new SafetyZoneHelper(ShoulderConstants.MinimumAngleDegrees, ShoulderConstants.MaximumAngleDegrees);
    }

    public Rotation2d getRotation() {
        if(Robot.isSimulation()) {
            return Rotation2d.fromRadians(m_armSim.getAngleRads());
        }
        return Rotation2d.fromDegrees((m_AbsoluteEncoder.getAbsolutePosition() * ShoulderConstants.AbsoluteAngleConversionFactor) + ShoulderConstants.AbsoluteAngleZeroOffset);
    }

    public void updateSafetyZones(ArmPose targetArmPose, double extenderLengthMeters, Rotation2d wristAngle) {
        if (extenderLengthMeters >= ExtenderConstants.ExtenderSafeLimit || wristAngle.getDegrees() >= WristConstants.MaximumSafeAngleDegrees || wristAngle.getDegrees() <= WristConstants.MinimumSafeAngleDegrees) {
            double normalizedCurrentAngle = normalize(getRotation());
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

    public void setTargetAngle(Rotation2d targetAngle) {
        double targetDegrees = normalize(targetAngle);
        targetDegrees = m_SafetyZoneHelper.getSafeValue(targetDegrees);
        m_targetDegrees = targetDegrees;
        m_shoulderL_A.getPIDController().setReference(targetDegrees, ControlType.kPosition);
        m_shoulderL_B.getPIDController().setReference(targetDegrees, ControlType.kPosition);
        m_shoulderR_A.getPIDController().setReference(targetDegrees, ControlType.kPosition);
        m_shoulderR_B.getPIDController().setReference(targetDegrees, ControlType.kPosition);
    }
    public static double normalize(double targetDegrees) {
        targetDegrees %= 360;
        if (targetDegrees <= -230) {
            targetDegrees += 360;
        } else if (targetDegrees >= 50) {
            targetDegrees -= 360;
        }
        return targetDegrees;
    } 
    // we have positions, degrees are associated with them

    public static double normalize(Rotation2d targetRotation2d) {
        return normalize(targetRotation2d.getDegrees());
    }


    public void tunePID(PIDUpdate pidUpdate) {
        setPID(pidUpdate, m_shoulderL_A.getPIDController());
        setPID(pidUpdate, m_shoulderL_B.getPIDController());
        setPID(pidUpdate, m_shoulderR_A.getPIDController());
        setPID(pidUpdate, m_shoulderR_B.getPIDController());
        m_simPid.setPID(pidUpdate.P, pidUpdate.I, pidUpdate.D);
    }

    public void setPID(PIDUpdate pidUpdate, SparkMaxPIDController controller) {
        controller.setP(pidUpdate.P);
        controller.setI(pidUpdate.I);
        controller.setD(pidUpdate.D);
    }
    
    private void initializeSparkMaxEncoder(CANSparkMax sparkMax, Rotation2d absoluteAngle) {
       RelativeEncoder encoder = sparkMax.getEncoder();
       encoder.setPositionConversionFactor(ShoulderConstants.SparkMaxEncoderConversionFactor);
       encoder.setPosition(absoluteAngle.getDegrees());
       Robot.logManager.addNumber("Shoulder/SparkMax" + sparkMax.getDeviceId() + "/TranslatedAngle", ()->encoder.getPosition());
    }

    public boolean atTarget(){
        if (Double.isNaN(m_targetDegrees)) {
            return false;
        }
        return Math.abs(getRotation().getDegrees() - m_targetDegrees) < ShoulderConstants.ToleranceDegrees;
    }

    private double getFeedForward(double currentExtensionMeters) {
        double slope = (ShoulderConstants.MaximumFeedForwardVoltage - ShoulderConstants.MinimumFeedForwardVoltage) / 
        (ExtenderConstants.MaximumPositionMeters - ExtenderConstants.MinimumPositionMeters);
        double offset = ShoulderConstants.MinimumFeedForwardVoltage - (ExtenderConstants.MinimumPositionMeters * slope);
        double maxFeedForward = (slope * currentExtensionMeters) + offset;
        return maxFeedForward * getRotation().getCos();
    }

    public void periodic(double currentExtensionMeters) {
        double feedForwardVoltage = getFeedForward(currentExtensionMeters);
        SmartDashboard.putNumber("shoulder/ff", feedForwardVoltage);
        m_shoulderL_A.getPIDController().setFF(feedForwardVoltage);
        m_shoulderL_B.getPIDController().setFF(feedForwardVoltage);
        m_shoulderR_A.getPIDController().setFF(feedForwardVoltage);
        m_shoulderR_B.getPIDController().setFF(feedForwardVoltage);
        m_pidTuner.tune();
        if (Robot.isSimulation()) {
            feedForwardVoltage = getFeedForward(kSimExtenderFixedPosition);
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
}
