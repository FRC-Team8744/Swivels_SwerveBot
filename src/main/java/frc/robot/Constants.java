// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class SwerveConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second (radians!)

    // The drive classes use the NWU axes convention (North-West-Up as external reference in the world frame).
    // The positive X axis points ahead, the positive Y axis points left, and the positive Z axis points up.
    // We use NWU here because the rest of the library, and math in general, use NWU axes convention.
    // https://docs.wpilib.org/en/stable/docs/software/hardware-apis/motors/wpi-drive-classes.html#axis-conventions
    public static final int kFrontLeftDriveMotorPort = 5;
    public static final int kFrontRightDriveMotorPort = 1;
    public static final int kRearLeftDriveMotorPort = 8;
    public static final int kRearRightDriveMotorPort = 4;

    public static final int kFrontLeftTurningMotorPort = 6;
    public static final int kFrontRightTurningMotorPort = 2;
    public static final int kRearLeftTurningMotorPort = 7;
    public static final int kRearRightTurningMotorPort = 3;

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kRearLeftTurningEncoderReversed = true;
    public static final boolean kRearRightTurningEncoderReversed = true;

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kRearLeftDriveEncoderReversed = true;
    public static final boolean kRearRightDriveEncoderReversed = true;

    public static final int kFrontLeftMagEncoderPort = 12;
    public static final int kFrontRightMagEncoderPort = 9;
    public static final int kRearLeftMagEncoderPort = 11;
    public static final int kRearRightMagEncoderPort = 10;

    public static final double kFrontLeftMagEncoderOffsetDegrees = 81.12;
    public static final double kFrontRightMagEncoderOffsetDegrees = 133.77;
    public static final double kRearLeftMagEncoderOffsetDegrees = 11.25;
    public static final double kRearRightMagEncoderOffsetDegrees = 66.71;

    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = Units.inchesToMeters(20.472);

    // Distance between front and back wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(20.472);

    public static final int kIMU_ID = 13;
    public static final boolean kGyroReversed = false;

    public static int kSwerveFL_enum = 0;
    public static int kSwerveFR_enum = 1;
    public static int kSwerveRL_enum = 2;
    public static int kSwerveRR_enum = 3;
  }

  public static final class ConstantsOffboard {
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    public static final double DRIVE_GEAR_RATIO = 6.75 / 1.0; // 6.75:1
    public static final double DRIVE_ROTATIONS_TO_METERS = WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO;
    public static final double DRIVE_RPM_TO_METERS_PER_SECOND = DRIVE_ROTATIONS_TO_METERS / 60.0;
    public static final double ANGLE_GEAR_RATIO = (150 / 7) / 1.0; // 150/7:1
    public static final double ANGLE_ROTATIONS_TO_RADIANS = (Math.PI * 2) / ANGLE_GEAR_RATIO;
    public static final double ANGLE_RPM_TO_RADIANS_PER_SECOND = DRIVE_ROTATIONS_TO_METERS / 60.0;

    /** Speed ramp. */
    public static final double OPEN_LOOP_RAMP = 0.25;
    public static final double CLOSED_LOOP_RAMP = 0.0;

    /** Current limiting. */
    public static final int DRIVE_CURRENT_LIMIT = 40;
    public static final int ANGLE_CURRENT_LIMIT = 20;

    /** Drive motor PID values. */
    public static final double DRIVE_KP = 0.1;
    public static final double DRIVE_KI = 0.0;
    public static final double DRIVE_KD = 0.0;
    public static final double DRIVE_KF = 0.0;

    /** Drive motor characterization. */
    public static final double DRIVE_KS = 0.11937;
    public static final double DRIVE_KV = 2.6335;
    public static final double DRIVE_KA = 0.46034;

    /** Angle motor PID values. */
    public static final double ANGLE_KP = 1.5;
    public static final double ANGLE_KI = 0.0;
    public static final double ANGLE_KD = 0.1;
    public static final double ANGLE_KF = 0.0;
    
    /** Swerve constraints. */
    public static final double MAX_SPEED_IN_PERCENT = 30.0;
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 0.0442 * MAX_SPEED_IN_PERCENT;
    public static final double MAX_ANGULAR_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND * 4/3;

    /** Inversions. */
    public static final boolean DRIVE_MOTOR_INVERSION = true;
    public static final boolean ANGLE_MOTOR_INVERSION = true;
    public static final boolean CANCODER_INVERSION = false;

    /** Idle modes. */
    public static final IdleMode DRIVE_IDLE_MODE = IdleMode.kBrake;
    public static final IdleMode ANGLE_IDLE_MODE = IdleMode.kBrake;

    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;
  }

  public static final class DriveConstants {
    public static final int kLeftFrontCAN = 9;
    public static final int kLeftRearCAN = 10;
    public static final int kRightFrontCAN = 7;
    public static final int kRightRearCAN = 8;

    public static final double kConvertInchToMeter = 0.0254;

    public static final double kTrackwidthInches = 18.5;
    public static final double kTrackwidthMeters = kTrackwidthInches * kConvertInchToMeter;

    public static final double kWheelDiameterInches = 6.0;
    public static final double kGearRatio = 8.45;
    public static final double kWheelDiameterMeters = kWheelDiameterInches * kConvertInchToMeter;
    public static final double kUnitsPerRotation = (kWheelDiameterMeters * Math.PI);  // !!! Use this value in SysID!
    public static final double kEncoderDistancePerRevolution = (kWheelDiameterMeters * Math.PI) / (kGearRatio);

    public static final double kEncoderVelocityFactor = kEncoderDistancePerRevolution/60.0; // Convert RPM to meters/second

    public static final double kMoveP = 1;
    public static final double kMoveI = 0;
    public static final double kMoveD = 0;

    // SparkMax Velocity PID
    public static final double kP = 0.00001;
    public static final double kI = 0;
    public static final double kD = 0; 
    public static final double kIz = 0; 
    public static final double kFF = 0.00018;
    public static final double kMaxOutput = 1; 
    public static final double kMinOutput = -1;
    // double maxRPM = 5700;

    public static final double kMaxDriveOutput = 0.4;  // range 0 to 1

    public static final double kMaxSpeedMetersPerSecond = 0.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.4;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;

    // !!!alh
    public static final int kButtonA = 1;
    public static final int kButtonB = 2;
    public static final int kButtonX = 3;
    public static final int kButtonY = 4;
    public static final int kButtonLeftBumper = 5;
    public static final int kButtonRightBumper = 6;
  }
}
