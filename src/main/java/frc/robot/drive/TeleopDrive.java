// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopDrive extends CommandBase {
    private final Joystick joystick;
    private final DriveSystem driveSystem;

    /** Creates a new TeleopDrive. */
    public TeleopDrive(final Joystick joystick, final DriveSystem driveSystem) {
        this.joystick = joystick;
        this.driveSystem = driveSystem;
        addRequirements(driveSystem);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        final var xSpeed = -joystick.getRawAxis(1);
        final var zRotation = joystick.getRawAxis(4);
        driveSystem.arcadeDrive(xSpeed, zRotation);
    }
}
