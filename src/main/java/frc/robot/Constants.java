// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class TestFixtureConstants {
    public static final int kDriveMotorPort = 1;
    public static final int kTurningMotorPort = 2;
    public static final int kAngleEncoderPort = 0;
    public static final int kfalconEncoderCPR = 2048;
    public static final double kSteeringGearRatio = 12.8;
    public static final double kEncoderTicksPerRevolution = kfalconEncoderCPR * kSteeringGearRatio;
    public static final double kTestVelocity = 5;
	  public static int kCANCoderID = 21;
  }

  public static final class DriveConstants {
    public static final int kFrontLeftDriveMotorPort = 1;
    public static final int kRearLeftDriveMotorPort = 4;
    public static final int kFrontRightDriveMotorPort = 2;
    public static final int kRearRightDriveMotorPort = 3;

    public static final int kFrontLeftTurningMotorPort = 5;
    public static final int kRearLeftTurningMotorPort = 8;
    public static final int kFrontRightTurningMotorPort = 6;
    public static final int kRearRightTurningMotorPort = 7;

    public static final int kFrontLeftTurningEncoderPort = 2;
    public static final int kRearLeftTurningEncoderPort = 3;
    public static final int kFrontRightTurningEncoderPort = 0;
    public static final int kRearRightTurningEncoderPort = 1;

    public static final boolean kFrontLeftTurningEncoderReversed = true;
    public static final boolean kRearLeftTurningEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = true;
    public static final boolean kRearRightTurningEncoderReversed = false;

    public static final int kFrontLeftDriveEncoderPort = 0;
    public static final int kRearLeftDriveEncoderPort = 0;
    public static final int kFrontRightDriveEncoderPort = 0;
    public static final int kRearRightDriveEncoderPort = 0;

    public static final boolean kFrontLeftDriveEncoderReversed = true;
    public static final boolean kRearLeftDriveEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = true;
    public static final boolean kRearRightDriveEncoderReversed = false;

    public static final double kTrackWidth = 0.5;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.7;
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2), //front left
            new Translation2d(-1*kWheelBase / 2, kTrackWidth / 2), //rear left
            new Translation2d(kWheelBase / 2, -1*kTrackWidth / 2), //front right
            new Translation2d(-1*kWheelBase / 2, -1*kTrackWidth / 2)); //rear right

    public static final boolean kGyroReversed = false;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The RobotPy Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public static final double ksVolts = 1;
    public static final double kvVoltSecondsPerMeter = 2;
    public static final double kaVoltSecondsSquaredPerMeter = 0.15;
    public static final double kWheelHeight = .1;//meters

    public static final double kMetersPerRevolution = kWheelHeight * Math.PI; // Circumfrence

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kBoostModifier = 1;
    
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPThetaController = 2;
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
      kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    public static final double kFrontRightOffset = 0.8934;
    public static final double kFrontLeftOffset = 0.9793;
    public static final double kRearRightOffset = 0.8768;
    public static final double kRearLeftOffset = 0.3515;
  }

  public static final class ModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 0.5 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 0.5 * Math.PI;
    public static double kUModuleTurningController = 1.8;
    public static double kPModuleTurningController = .15*kUModuleTurningController;
    public static double tUModuleTurningController = .5; // Seconds
    public static double kIModuleTurningController =0 * (.4*kUModuleTurningController)/tUModuleTurningController;
    public static double kDModuleTurningController = 0 * 0.0666666666*kUModuleTurningController*tUModuleTurningController;

    public static final double kPModuleDriveController = 0; // 0.15;
    public static final double kDModuleDriveController = 0;
    public static final double kIModuleDriveController = 0; // 0.3;

    public static final double MaxAllowableError = 0; //encoder ticks
    public static final int kfalconEncoderCPR = 2048;
    public static final double kSteeringGearRatio = 12.8;
    public static final double kDriveGearRatio = 6.75;

    public static final double kSteerEncoderTicksPerRevolution = kfalconEncoderCPR * kSteeringGearRatio;
    public static final double kDriveEncoderTicksPerRevolution = kfalconEncoderCPR * kDriveGearRatio;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1.125;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.5;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 2;
    public static final double kPYController = 2;
    public static final double kPThetaController = 2;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
