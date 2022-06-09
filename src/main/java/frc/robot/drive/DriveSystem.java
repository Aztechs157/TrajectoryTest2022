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
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSystem extends SubsystemBase {

    // #region Motors
    private final CANSparkMax motorLeft1 = new CANSparkMax(Constants.kMotorLeft1, MotorType.kBrushless);
    private final CANSparkMax motorLeft2 = new CANSparkMax(Constants.kMotorLeft2, MotorType.kBrushless);
    private final MotorControllerGroup motorsLeft = new MotorControllerGroup(motorLeft1, motorLeft2);
    {
        motorsLeft.setInverted(false);
    }

    private final CANSparkMax motorRight1 = new CANSparkMax(Constants.kMotorRight1, MotorType.kBrushless);
    private final CANSparkMax motorRight2 = new CANSparkMax(Constants.kMotorRight2, MotorType.kBrushless);
    private final MotorControllerGroup motorsRight = new MotorControllerGroup(motorRight1, motorRight2);
    {
        motorsRight.setInverted(true);
    }

    private final DifferentialDrive drive = new DifferentialDrive(motorsLeft, motorsRight);

    public void tankDriveVolts(final double leftVolts, final double rightVolts) {
        motorsLeft.setVoltage(leftVolts);
        motorsRight.setVoltage(rightVolts);
        drive.feed();
    }
    // #endregion

    // #region Encoders
    // This only uses the front two encoders for now,
    // posibally average both the front and back later?
    private final RelativeEncoder encoderLeft = motorLeft1.getEncoder();
    private final RelativeEncoder encoderRight = motorRight1.getEncoder();
    {
        // REV encoders don't have setDistancePerPulse()
        // find replacment
    }

    public void resetEncoders() {
        encoderLeft.setPosition(0);
        encoderRight.setPosition(0);
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(encoderLeft.getVelocity(), encoderRight.getVelocity());
    }

    public double getAverageEncoderDistance() {
        return (encoderLeft.getPosition() + encoderRight.getPosition()) / 2.0;
    }

    // #endregion

    // #region Gyro
    private final Gyro gyro = new ADXRS450_Gyro();

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return gyro.getRotation2d().getDegrees();
    }

    public double getTurnRate() {
        return -gyro.getRate();
    }
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
