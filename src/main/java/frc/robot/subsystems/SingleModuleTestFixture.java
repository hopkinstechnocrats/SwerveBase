// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.TestFixtureConstants;

public class SingleModuleTestFixture extends SubsystemBase {
  /** Creates a new SingleModuleTestFixture. */

  private final WPI_TalonFX driveMotor = new WPI_TalonFX(TestFixtureConstants.kDriveMotorPort);
  private final WPI_TalonFX turningMotor = new WPI_TalonFX(TestFixtureConstants.kTurningMotorPort);
  private final DigitalSource encoderPWMPort = new DigitalInput(TestFixtureConstants.kAngleEncoderPort);
  private final DutyCycle encoderPWM = new DutyCycle(encoderPWMPort);
  private final DutyCycleEncoder angleEncoder = new DutyCycleEncoder(encoderPWM);
  private final PIDController m_turningPIDController =
      new PIDController(
          ModuleConstants.kPModuleTurningController,
          0,
          0
          // new TrapezoidProfile.Constraints(
          //     ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
          //     ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared)
          );

  private Rotation2d swerveAngle;

  public SingleModuleTestFixture() {
    driveMotor.setNeutralMode(NeutralMode.Brake);
    turningMotor.setNeutralMode(NeutralMode.Brake);
    m_turningPIDController.enableContinuousInput(0, 2*Math.PI);
    swerveAngle = new Rotation2d(0,1);
  }

  public void setState(double driveSpeed, double turnSpeed) {
    driveMotor.set(driveSpeed/2);
    System.out.println("Setting Drive Motor To "+driveSpeed);
    // turningMotor.set(turnSpeed/2);
    // System.out.println("Setting Turning Motor To "+driveSpeed);
  }

  public void setAngle(Rotation2d desiredAngle) {
    swerveAngle = desiredAngle;
  }

  public double getRevolutions() {
    return this.angleEncoder.get();
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(getRevolutions() * 360 % 360);
  }

  @Override
  public void periodic() {
    System.out.println(this.getAngle());
    SmartDashboard.putNumber("SwerveEncoderRaw", this.getRevolutions());
    SmartDashboard.putNumber("SwerveAngle", this.getAngle().getDegrees());
    SmartDashboard.putNumber("EncoderFrequency", this.encoderPWM.getFrequency());
    SmartDashboard.putNumber("EncoderOutput", this.encoderPWM.getOutput());
    SmartDashboard.putNumber("PIDSetpoint", m_turningPIDController.getSetpoint());
    SmartDashboard.putNumber("PIDInput", m_turningPIDController.getPositionError());
    final var turnOutput =
        m_turningPIDController.calculate(getAngle().getRadians(), swerveAngle.getRadians());
    turningMotor.set(turnOutput);
    turningMotor.feed();
    // This method will be called once per scheduler run
  }
}
