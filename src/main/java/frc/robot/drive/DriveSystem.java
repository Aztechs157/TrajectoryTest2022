// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSystem extends SubsystemBase {
    private final ShuffleboardTab tab = Shuffleboard.getTab("Drive");

    // #region Motors
    private final CANSparkMax motorLeft = new CANSparkMax(Constants.kMotorLeft, MotorType.kBrushless);
    private final CANSparkMax motorRight = new CANSparkMax(Constants.kMotorRight, MotorType.kBrushless);
    {
        motorLeft.setInverted(true);
        motorRight.setInverted(false);
    }

    private final DifferentialDrive drive = new DifferentialDrive(motorLeft, motorRight);
    {
        tab.add("Drive", drive);
    }

    public void tankDriveVolts(final double leftVolts, final double rightVolts) {
        motorLeft.setVoltage(leftVolts);
        motorRight.setVoltage(rightVolts);
        drive.feed();
    }

    public void arcadeDrive(final double xSpeed, final double zRotation) {
        drive.arcadeDrive(xSpeed, zRotation);
    }

    public void tankDrive(final double leftSpeed, final double rightSpeed) {
        drive.tankDrive(leftSpeed, rightSpeed);
    }
    // #endregion

    // #region Encoders
    // This only uses the front two encoders for now,
    // posibally average both the front and back later?
    private final RelativeEncoder encoderLeft = motorLeft.getEncoder();
    private final RelativeEncoder encoderRight = motorRight.getEncoder();
    {
        encoderLeft.setPositionConversionFactor(Constants.kRotationsToMeters);
        encoderRight.setPositionConversionFactor(Constants.kRotationsToMeters);

        tab.addNumber("Left Encoder", encoderLeft::getPosition);
        tab.addNumber("Right Encoder", encoderRight::getPosition);
        tab.add("Reset Encoders", new InstantCommand(this::resetEncoders));
    }

    public void resetEncoders() {
        encoderLeft.setPosition(0);
        encoderRight.setPosition(0);
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(encoderLeft.getVelocity(), encoderRight.getVelocity());
    }
    // #endregion

    // #region Gyro
    private final Gyro gyro = new ADXRS450_Gyro();
    // #endregion

    // #region Odometry
    private final DifferentialDriveOdometry odometry;
    {
        resetEncoders();
        odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
    }

    @Override
    public void periodic() {
        odometry.update(gyro.getRotation2d(), encoderLeft.getPosition(), encoderRight.getPosition());
    }

    public void resetOdometry(final Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(pose, gyro.getRotation2d());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }
    // #endregion
}
