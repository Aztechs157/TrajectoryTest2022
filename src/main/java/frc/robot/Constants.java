// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /** Volts */
    public static final double kFeedfowardS = 0.22;
    /** VoltSecondsPerMeter */
    public static final double kFeedfowardV = 1.98;
    /** VoltSecondsSquaredPerMeter */
    public static final double kFeedfowardA = 0.2;

    public static final double kDriveVelocityP = 8.5;

    public static final double kWheelRadius = 0;

    /** Meters */
    public static final double kTrackwidth = 0.69;
    public static final DifferentialDriveKinematics kDriveKinematics = //
            new DifferentialDriveKinematics(kTrackwidth);

    /** MetersPerSecond */
    public static final double kMaxVelocity = 3;
    /** MetersPerSecondSquared */
    public static final double kMaxAcceleration = 3;

    public static final double kMaxVoltage = 10;

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final int kMotorLeft = 2;
    public static final int kMotorRight = 3;

    public static final double kTeleopPercent = 0.5;

    public static final double kRotationsToMeters = 0.05346700048;
    public static final double kRPMToMetersPerSecond = kRotationsToMeters / 60;
}
