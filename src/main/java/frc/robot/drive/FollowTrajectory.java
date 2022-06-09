// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

public class FollowTrajectory extends SequentialCommandGroup {

    public FollowTrajectory(final DriveSystem driveSystem, final Trajectory trajectory) {

        final var ramseteCommand = new RamseteCommand(
                trajectory,
                driveSystem::getPose,
                new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
                new SimpleMotorFeedforward(
                        Constants.kFeedfowardS,
                        Constants.kFeedfowardV,
                        Constants.kFeedfowardA),
                Constants.kDriveKinematics,
                driveSystem::getWheelSpeeds,
                new PIDController(Constants.kDriveVelocityP, 0, 0),
                new PIDController(Constants.kDriveVelocityP, 0, 0),
                // RamseteCommand passes volts to the callback
                driveSystem::tankDriveVolts,
                driveSystem);

        // Reset odometry to the starting pose of the trajectory.
        driveSystem.resetOdometry(trajectory.getInitialPose());

        // Run path following command, then stop at the end.
        addCommands(ramseteCommand, new StopDriving(driveSystem));
    }
}
