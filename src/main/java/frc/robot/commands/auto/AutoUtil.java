package frc.robot.commands.auto;

import com.chaos131.auto.ParsedCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmPose;
import frc.robot.subsystems.arm.Wrist.CoordinateType;
import frc.robot.subsystems.swerve.DrivePose;

public class AutoUtil {
    public static double ParseDouble(String stringToParse, double defaultValue) {
        if(stringToParse == null) {
            return defaultValue;
        }

        return Double.parseDouble(stringToParse);
    }
    
    /**
     * Gets the drive translation tolerance from the auto script step
     * @param parsedCommand the auto script step
     */
    public static double getTranslationTolerance(ParsedCommand parsedCommand) {
        return AutoUtil.ParseDouble(parsedCommand.getArgument("translationTolerance"), Constants.DriveToTargetTolerance);
    }

    /**
     * Gets the drive pose from the auto script step (if pose doesn't exist, it returns null)
     * @param parsedCommand the auto script step
     */
    public static Pose2d getDrivePose(ParsedCommand parsedCommand) {
        var poseName = parsedCommand.getArgument("drivePose");
        if(poseName != null && DrivePose.DrivePoses.containsKey(poseName)) {
          return DrivePose.DrivePoses.get(poseName).getCurrentAlliancePose();
        } else if (poseName != null) {
            // If drive pose is asked for, but we don't match anything - don't return a pose.
            return null;
        } 
        double x_meters = AutoUtil.ParseDouble(parsedCommand.getArgument("x"), 0.0);
        double y_meters = AutoUtil.ParseDouble(parsedCommand.getArgument("y"), 0.0);
        double angle_degrees = AutoUtil.ParseDouble(parsedCommand.getArgument("angle"), 0.0);
        return DrivePose.toCurrentAlliancePose(new Pose2d(x_meters, y_meters, Rotation2d.fromDegrees(angle_degrees)));
    }

    /**
     * Gets the arm pose from the auto script step (if pose doesn't exist, it returns null)
     * @param parsedCommand the auto script step
     */
    public static ArmPose getArmPose(ParsedCommand parsedCommand) {
        var poseName = parsedCommand.getArgument("armPose");
        if(poseName != null && ArmPose.ArmPoses.containsKey(poseName)) {
          return ArmPose.ArmPoses.get(poseName);
        } else if (poseName != null) {
            // If arm pose is asked for, but we don't match anything - don't return a pose.
            return null;
        } 
        double shoulderDegrees = AutoUtil.ParseDouble(parsedCommand.getArgument("shoulder"), ArmPose.StowedPose.shoulderAngle.getDegrees());
        double extenderMeters = AutoUtil.ParseDouble(parsedCommand.getArgument("extender"), ArmPose.StowedPose.extenderPos);
        double wristDegrees = AutoUtil.ParseDouble(parsedCommand.getArgument("wrist"), ArmPose.StowedPose.wristAngle.getDegrees());
        return new ArmPose(shoulderDegrees, extenderMeters, wristDegrees, CoordinateType.ArmRelative);
    }
}
