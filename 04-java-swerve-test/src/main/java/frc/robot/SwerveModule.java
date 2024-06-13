// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import com.ctre.phoenix6.hardware.TalonFX;


public class SwerveModule {
  private static final double kWheelRadius = 0.0508;

  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration =
      2 * Math.PI; // radians per second squared

  // private final PWMSparkMax m_driveMotor;
  // private final PWMSparkMax m_azimuthMotor;

  private final TalonFX m_driveMotor;
  private final TalonFX m_azimuthMotor;

  private final double m_driveRatio;
  private final double m_azimuthRatio;

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(0.01, 0, 0);

  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController m_azimuthPIDController =
      new ProfiledPIDController(
          0.01,
          0,
          0,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
  private final SimpleMotorFeedforward m_azimuthFeedforward = new SimpleMotorFeedforward(1, 0.5);

  /**
   * Constructs a SwerveModule with a drive motor, azimuth motor, drive encoder and azimuth encoder.
   *
   * @param driveMotorID CAN ID for the drive motor
   * @param azimuthMotorID CAN ID for the azimuth motor
   * @param driveGearRatio Gear ratio for the drive motor
   * @param azimuthGearRatio Gear ratio for the azimuth motor
   */
  public SwerveModule(
      int driveMotorID,
      int azimuthMotorID,
      double driveGearRatio,
      double azimuthGearRatio) {
    m_driveMotor = new TalonFX(driveMotorID, "canivore");
    m_azimuthMotor = new TalonFX(azimuthMotorID, "canivore");

    m_driveRatio = driveGearRatio;
    m_azimuthRatio = azimuthGearRatio;

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_azimuthPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveMotor.getRotorVelocity().getValueAsDouble()*m_driveRatio*2*Math.PI*kWheelRadius, new Rotation2d(m_azimuthMotor.getRotorPosition().getValueAsDouble()*m_azimuthRatio));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveMotor.getRotorPosition().getValueAsDouble()*m_driveRatio*2*Math.PI*kWheelRadius, new Rotation2d(m_azimuthMotor.getRotorPosition().getValueAsDouble()*m_azimuthRatio));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = new Rotation2d(m_azimuthMotor.getRotorPosition().getValueAsDouble()*m_azimuthRatio);

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(m_driveMotor.getRotorVelocity().getValueAsDouble()*m_driveRatio*2*Math.PI*kWheelRadius, state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the azimuth motor output from the azimuth PID controller.
    final double azimuthOutput =
        m_azimuthPIDController.calculate(m_azimuthMotor.getRotorPosition().getValueAsDouble()*m_azimuthRatio, state.angle.getRadians());

    final double azimuthFeedforward =
        m_azimuthFeedforward.calculate(m_azimuthPIDController.getSetpoint().velocity);

    m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    m_azimuthMotor.setVoltage(azimuthOutput + azimuthFeedforward);
  }
}
