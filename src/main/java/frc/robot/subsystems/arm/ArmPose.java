// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.ArmConstants.ExtenderConstants;
import frc.robot.subsystems.arm.Wrist.CoordinateType;

/** Add your docs here. */
public class ArmPose {
    public final Rotation2d shoulderAngle;
    public final double extenderPos;
    public final Rotation2d wristAngle;
    public final CoordinateType wristCoordinate;

    public ArmPose(Rotation2d shoulderAngle, double extenderPos, Rotation2d wristAngle, CoordinateType wristCoordinate){
        this.shoulderAngle = shoulderAngle;
        this.extenderPos = extenderPos;
        this.wristAngle = wristAngle;
        this.wristCoordinate = wristCoordinate;
    }

    public ArmPose(double shoulderAngleDegrees, double extenderPos, double wristAngleDegrees, CoordinateType wristCoordinate){
        this(Rotation2d.fromDegrees(shoulderAngleDegrees), extenderPos, Rotation2d.fromDegrees(wristAngleDegrees), wristCoordinate);
    }

    private static Map<String, ArmPose> ArmPoses = new HashMap<String, ArmPose>();
    private static ArmPose addStaticPose(String name, double shoulderAngleDegrees, double extenderPos, double wristAngleDegrees, CoordinateType wristCoordinate) {
        var pose = new ArmPose(shoulderAngleDegrees, extenderPos, wristAngleDegrees, wristCoordinate);
        ArmPoses.put(name, pose);
        return pose;
    }

    //Stowed Pose
    public static final ArmPose StowedPose = addStaticPose("Stowed", -90, ExtenderConstants.MinimumPositionMeters, 180, CoordinateType.ArmRelative);
    
    //Pick Poses
    public static final ArmPose PickFlatPose = addStaticPose("PickFlatPose", -180, ExtenderConstants.MinimumPositionMeters + 0.5, -10, CoordinateType.ArmRelative);
    public static final ArmPose PickWithWristUpPose = addStaticPose("PickWithWristUpPose", -180, ExtenderConstants.MinimumPositionMeters + 0.5, 330, CoordinateType.ArmRelative);

    //Intake Poses
    public static final ArmPose IntakeFlatPose = addStaticPose("IntakeFlatPose", -130, ExtenderConstants.MinimumPositionMeters + 0.5, 305, CoordinateType.ArmRelative);
    public static final ArmPose IntakeWithWristDownPose = addStaticPose("IntakeWithWristDownPose", -130, ExtenderConstants.MinimumPositionMeters + 0.25, 35, CoordinateType.ArmRelative);

    //Cube Poses
    public static final ArmPose CubeLowPose = addStaticPose("CubeLowPose", -40, ExtenderConstants.MinimumPositionMeters + 0.1, 320, CoordinateType.ArmRelative);
    public static final ArmPose CubeMidPose = addStaticPose("CubeMidPose", -10, ExtenderConstants.MinimumPositionMeters + 0.2, 310, CoordinateType.ArmRelative);
    public static final ArmPose CubeHighPose = addStaticPose("CubeHighPose", 10, ExtenderConstants.MinimumPositionMeters + 0.5, 310, CoordinateType.ArmRelative);

    //Cone Poses
    public static final ArmPose ConeLowPose = addStaticPose("ConeLowPose", -50, ExtenderConstants.MinimumPositionMeters + 0.1, 53, CoordinateType.ArmRelative);
    public static final ArmPose ConeMidPose = addStaticPose("ConeMidPose", 5, ExtenderConstants.MinimumPositionMeters + 0.15, 0, CoordinateType.ArmRelative);
    public static final ArmPose ConeHighPose = addStaticPose("ConeHighPose", 15, ExtenderConstants.MinimumPositionMeters + 1.32, -5, CoordinateType.ArmRelative);

    //Test Poses
    public static final ArmPose TopRightTestPose = addStaticPose("TopRight", 45, ExtenderConstants.MaximumPositionMeters, 0, CoordinateType.ArmRelative);
    public static final ArmPose TopLeftTestPose = addStaticPose("TopLeft", -225, ExtenderConstants.MaximumPositionMeters, 0, CoordinateType.ArmRelative);
    public static final ArmPose BottomRightTestPose = addStaticPose("BottomRight", -45, ExtenderConstants.MaximumPositionMeters, 0, CoordinateType.ArmRelative);
    public static final ArmPose BottomLeftTestPose = addStaticPose("BottomLeft", -135, ExtenderConstants.MaximumPositionMeters, 0, CoordinateType.ArmRelative);


    public static void forAllPoses(BiConsumer<String, ArmPose> lambdaFunction) {
        ArmPoses.forEach(lambdaFunction);
    }
}