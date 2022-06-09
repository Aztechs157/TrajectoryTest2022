// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import edu.wpi.first.wpilibj2.command.RunCommand;

public class StopDriving extends RunCommand {

    /** Creates a new StopDriving. */
    public StopDriving(final DriveSystem driveSystem) {
        super(() -> driveSystem.tankDriveVolts(0, 0), driveSystem);
    }
}
