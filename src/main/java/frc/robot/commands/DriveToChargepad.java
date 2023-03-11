package frc.robot.commands;

import java.sql.Driver;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDrive;

public class DriveToChargepad extends DriveToTarget {
    public DriveToChargepad(SwerveDrive swerveDrive, double translationTolerance) {
        super(swerveDrive, translationTolerance);
        m_poseSupplier = (DriverStation.getAlliance() == DriverStation.Alliance.Blue) ? () -> new Pose2d(0, 0, swerveDrive.getGyroRotation()) : () -> new Pose2d(0, 0, swerveDrive.getGyroRotation()); // TODO find these values
        m_swerveDrive = swerveDrive;
        addRequirements(m_swerveDrive);
    }
    
    
}
