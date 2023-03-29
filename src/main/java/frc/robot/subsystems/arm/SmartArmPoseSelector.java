package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class SmartArmPoseSelector {
    public enum PoseType{
        score(-180, 0),
        singleSub(-90, -90),
        doubleSub(0, -180);

        Rotation2d m_redAngle;
        Rotation2d m_blueAngle;
        PoseType(double redAngleDegrees, double blueAngleDegrees) {
            m_redAngle = Rotation2d.fromDegrees(redAngleDegrees);
            m_blueAngle = Rotation2d.fromDegrees(blueAngleDegrees);
        }
    }
    private ArmPose m_frontPose;
    private ArmPose m_backPose;
    private PoseType m_poseType;

    SmartArmPoseSelector(ArmPose frontPose, ArmPose backPose, PoseType poseType) {
        m_frontPose = frontPose;
        m_backPose = backPose;
        m_poseType = poseType;
    }

    public ArmPose getSmartPose(Rotation2d robotFieldRotation) {
        Rotation2d targetAngle = DriverStation.getAlliance() == Alliance.Red ? m_poseType.m_redAngle : m_poseType.m_blueAngle;
        if (Math.abs(targetAngle.minus(robotFieldRotation).getDegrees()) < 90) {
            return m_frontPose;
        } else{
            return m_backPose;
        }
    }
}
