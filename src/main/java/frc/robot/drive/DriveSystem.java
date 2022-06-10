// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSystem extends SubsystemBase {
    private static final double kCountsPerRevolution = 1440.0;
    private static final double kWheelDiameterInch = 2.75591; // 70 mm

    private final ShuffleboardTab tab = Shuffleboard.getTab("Drive");

    // The Romi has the left and right motors set to
    // PWM channels 0 and 1 respectively
    private final Spark m_leftMotor = new Spark(0);
    private final Spark m_rightMotor = new Spark(1);
    {
        m_leftMotor.setInverted(false);
        m_rightMotor.setInverted(true);
    }

    // Set up the differential drive controller
    private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotor, m_rightMotor);

    public void arcadeDrive(final double xSpeed, final double zRotate) {
        m_drive.arcadeDrive(xSpeed, zRotate);
    }

    // The Romi has onboard encoders that are hardcoded
    // to use DIO pins 4/5 and 6/7 for the left and right
    private final Encoder m_leftEncoder = new Encoder(4, 5);
    private final Encoder m_rightEncoder = new Encoder(6, 7);
    {
        // Use inches as unit for encoder distances
        m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
        m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);

        tab.add("Left Encoder", m_leftEncoder);
        tab.add("Right Encoder", m_rightEncoder);
        tab.add("Reset Encoders", new InstantCommand(this::resetEncoders));
    }

    public void resetEncoders() {
        m_leftEncoder.reset();
        m_rightEncoder.reset();
    }
}
