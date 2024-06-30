// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// test

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoublePublisher;

public class SwerveModule {
  // Network table communication
  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable table = inst.getTable("datatable");

  // SwerveModule Drive Values for Network Table
  private DoublePublisher driveOutputNT = table.getDoubleTopic("driveOutput").publish();
  private DoublePublisher driveFeedForwardNT = table.getDoubleTopic("driveFeedForward").publish();
  private DoublePublisher driveDesiredStateSpeed = table.getDoubleTopic("driveDesiredStateSpeed").publish();
  private DoublePublisher driveCurrentStateSpeed = table.getDoubleTopic("driveCurrentStateSpeed").publish();

  // SwerveModule Azimuth Values for Network Table
  private DoublePublisher azimuthOutputNT = table.getDoubleTopic("azimuthOutputNT").publish();
  private DoublePublisher azimuthFeedForwardNT = table.getDoubleTopic("azimuthFeedForwardNT").publish();
  private DoublePublisher azimuthDesiredStateAngle = table.getDoubleTopic("azimuthDesiredStateAngle").publish();
  private DoublePublisher azimuthCurrentStateAngle = table.getDoubleTopic("azimuthCurrentStateAngle").publish();

  // Constants for Swerve Module Characteristics
  private static final double kWheelRadius = 0.0636; // Wheel radius in Meters (2.5 inches)
  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared
  private final double m_driveRatio;
  private final double m_azimuthRatio;

  // Swerve Module Motor Definitions
  private final TalonFX m_driveMotor;
  private final TalonFX m_azimuthMotor;

  // Gains tuned using ZN method (tuned on holicanoli without a load)
  private final PIDController m_drivePIDController = new PIDController(0.07386364, 0.4166666, 0.0);

  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController m_azimuthPIDController =
      new ProfiledPIDController(
          0.1,
          0.0,
          0.0,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0.1, 0.3);
  private final SimpleMotorFeedforward m_azimuthFeedforward = new SimpleMotorFeedforward(0.1, 0.5);

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
      double azimuthGearRatio,
      TalonFXConfiguration driveMotorConfigs) {
    // Initialize Motors
    // NOTE: These are just setup for Holicanoli. Uncomment lines 82 and 83 when testing with the real robot
    // m_driveMotor = new TalonFX(driveMotorID, "canivore");
    // m_azimuthMotor = new TalonFX(azimuthMotorID, "rio");
    m_driveMotor = new TalonFX(driveMotorID, "canivore");
    m_azimuthMotor = new TalonFX(azimuthMotorID, "canivore");

    // Set drive motor configs
    m_driveMotor.getConfigurator().apply(driveMotorConfigs);

    m_driveRatio = driveGearRatio;
    m_azimuthRatio = azimuthGearRatio;

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_azimuthPIDController.enableContinuousInput(-Math.PI, Math.PI);
    m_azimuthMotor.setPosition(0.0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    /* SwerveModuleState
     * expects the following:
     * Double speedMetersPerSecond - The speed of the wheel of the module.
     * Rotation2d angle - The angle of the module.
     */

    /* Calculating speedMetersPerSecond
     * gerRotorVelocity - returns RPS (rotations per second)
     * m_driveRatio - is the gear ratio for the drive motor should be 7:1, because of this I think we need to divide instead of multiply
     * kWheelRadius - radius of the wheel in meters
     * 1. Convert motor shaft rps to output shaft (wheel) rps = m_driveMotor.getRotorVelocity().getValueAsDouble() / m_driveRatio
     * 2. Calculate wheel distance per rotation = 2 * Math.PI * kWheelRadius
     * 3. multiply results from step 1 and step 2
     */

     /* Calculating angle
     * Rotation2d is expecting a value of radians (I am assuming from the wheels perspective).
     * To achieve this, we would need the following:
     * getRotorPosition - returns the Position of the motor rotor. (rotations).
     * m_azimuthRatio - is the gear ratio for the azimuth motor should be 20:1, because of this I think we need to divide instead of multiply
     * 1. Convert motor shaft angle to output shaft (wheel) angle = m_azimuthMotor.getRotorPosition().getValueAsDouble() / m_azimuthRatio
     * 2. Convert this value to radians (multiply result from step 1 by 2 * PI)
     */
    return new SwerveModuleState(
      (m_driveMotor.getRotorVelocity().getValueAsDouble() / m_driveRatio) * (2 * Math.PI * kWheelRadius),
      new Rotation2d((m_azimuthMotor.getRotorPosition().getValueAsDouble() / m_azimuthRatio) * 2 * Math.PI)
    );
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    /* SwerveModulePosition
     * expects the following:
     * Double distanceMeters - The distance measured by the wheel of the module.
     * Rotation2d angle - The angle of the module.
     */

     /* Calculating distanceMeters
     * getRotorPosition - returns rotations of rotor
     * m_driveRatio - is the gear ratio for the drive motor should be 7:1, because of this I think we need to divide instead of multiply
     * kWheelRadius - radius of the wheel in meters
     * 1. Convert motor shaft rotations to output shaft (wheel) rotations = m_driveMotor.getRotorPosition().getValueAsDouble() / m_driveRatio
     * 2. Calculate wheel distance per rotation = 2 * Math.PI * kWheelRadius
     * 3. multiply results from step 1 and step 2
     */

     /* Calculating angle
     * Rotation2d is expecting a value of radians (I am assuming from the wheels perspective).
     * To achieve this, we would need the following:
     * getRotorPosition - returns the Position of the motor rotor. (rotations).
     * m_azimuthRatio - is the gear ratio for the azimuth motor should be 20:1, because of this I think we need to divide instead of multiply
     * 1. Convert motor shaft angle to output shaft (wheel) angle = m_azimuthMotor.getRotorPosition().getValueAsDouble() / m_azimuthRatio
     * 2. Convert this value to radians (multiply result from step 1 by 2 * PI)
     */
    return new SwerveModulePosition(
      (m_driveMotor.getRotorPosition().getValueAsDouble() / m_driveRatio) * (2 * Math.PI * kWheelRadius),
      new Rotation2d((m_azimuthMotor.getRotorPosition().getValueAsDouble() / m_azimuthRatio) * 2 * Math.PI)
    );
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Get the encoder rotation in radians
    var encoderRotation = new Rotation2d((m_azimuthMotor.getRotorPosition().getValueAsDouble() / m_azimuthRatio) * 2 * Math.PI);

    // Optimize the reference state to avoid spinning further than PI radians (90 degrees)
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

    // Calculate the drive output from the drive PID controller.
    // Clamp the output to be between -1 and +1
    // final double driveOutput = state.speedMetersPerSecond / 5.8;
    final double driveOutput = MathUtil.clamp(
      m_drivePIDController.calculate(
        (m_driveMotor.getRotorVelocity().getValueAsDouble() / m_driveRatio) * (2 * Math.PI * kWheelRadius),
        state.speedMetersPerSecond
      ),
      -1.0, 
      1.0
    );

    // Do we need to make the clamp above -0.6 to +0.6 based on the ks and kv values set for SimpleMotorFeedForward?
    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the azimuth motor output from the azimuth PID controller.
    final double azimuthOutput = MathUtil.clamp(
      m_azimuthPIDController.calculate(
        (m_azimuthMotor.getRotorPosition().getValueAsDouble() / m_azimuthRatio) * 2 * Math.PI,
        state.angle.getRadians()
      ),
      -1.0,
      1.0
    );

    final double azimuthFeedforward = m_azimuthFeedforward.calculate(m_azimuthPIDController.getSetpoint().velocity);

    // For testing purposes we are only going to test the Holicanoli drive motors first
    if (m_azimuthMotor.getDeviceID() == 22) {
      // Output values to the network table to trend
      driveOutputNT.set(driveOutput);
      driveFeedForwardNT.set(driveFeedforward);
      driveDesiredStateSpeed.set(state.speedMetersPerSecond);
      driveCurrentStateSpeed.set(getState().speedMetersPerSecond);               

      azimuthOutputNT.set(azimuthOutput);
      azimuthFeedForwardNT.set(azimuthFeedforward);
      azimuthDesiredStateAngle.set(state.angle.getDegrees());
      azimuthCurrentStateAngle.set(getState().angle.getDegrees());
      // Take PID calculated commands for the motor and send it
      // m_azimuthMotor.set(azimuthOutput + 0);
    };
    
    // NOTE: Uncomment below code for testing on the real robot
    m_driveMotor.set(driveOutput + driveFeedforward);
    m_azimuthMotor.set(azimuthOutput);
  }
}
