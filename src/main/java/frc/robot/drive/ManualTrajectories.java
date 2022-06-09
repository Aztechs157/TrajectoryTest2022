// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import java.util.List;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Constants;

public class ManualTrajectories {
    /**
     * Create a trajectory that goes foward
     *
     * @return The trajectory
     */
    public static Trajectory goFoward() {
        return TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(),
                new Pose2d(1, 0, new Rotation2d(0)),
                defaultConfig());
    }

    /**
     * Create a trajectory that follows a criss cross path
     *
     * @return The trajectory
     */
    public static Trajectory crissCross() {
        // An example trajectory to follow. All units in meters.
        return TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                // Pass config
                defaultConfig());
    }

    /**
     * Generate a default trajectory config. Helper method for the trajectory
     * generating static methods of this class.
     *
     * @return
     */
    private static TrajectoryConfig defaultConfig() {
        final var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                        Constants.kFeedfowardS,
                        Constants.kFeedfowardV,
                        Constants.kFeedfowardA),
                Constants.kDriveKinematics,
                Constants.kMaxVoltage);

        // Create config for trajectory
        return new TrajectoryConfig(
                Constants.kMaxVelocity,
                Constants.kMaxAcceleration)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint);
    }
}
