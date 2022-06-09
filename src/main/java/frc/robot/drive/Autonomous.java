// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SelectCommand;

public class Autonomous {
    private final SendableChooser<Trajectory> chooser = new SendableChooser<>();
    private boolean hasDefault = false;
    {
        SmartDashboard.putData("Auto Chooser", chooser);
    }

    /**
     * Generate a command from this Autonomous to be used in
     * {@link RobotContainer#getAutonomousCommand()} that runs whatever trajectory
     * that is selected when called.
     *
     * @param driveSystem The drive system to pass to {@link FollowTrajectory}
     * @return The command
     */
    public SelectCommand getCommand(final DriveSystem driveSystem) {
        // SelectCommand allows us to delay creating the command until auto is started,
        // allowing us to select a trajaectory while disabled
        return new SelectCommand(() -> {
            final var trajectory = chooser.getSelected();
            return new FollowTrajectory(driveSystem, trajectory);
        });
    }

    /**
     * Add new option to this Autonomous. First option added is automatically the
     * default.
     *
     * @param name       The label on shuffleboard
     * @param trajectory The trajectory to follow
     */
    public void add(final String name, final Trajectory trajectory) {
        if (hasDefault) {
            chooser.addOption(name, trajectory);
        } else {
            chooser.setDefaultOption(name, trajectory);
        }
    }
}
