// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.hardware.Pigeon2;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  public static final double kMaxSpeed = 6.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final Translation2d m_module0Location = new Translation2d(0.254, -0.311);
  private final Translation2d m_module1Location = new Translation2d(0.254, 0.311);
  private final Translation2d m_module2Location = new Translation2d(-0.254, 0.311);
  private final Translation2d m_module3Location = new Translation2d(-0.254, -0.311);

  public static final double driveGearRatio = 7;
  public static final double azimuthGearRatio = 20;

  private final SwerveModule m_module0 = new SwerveModule(10, 20, driveGearRatio, azimuthGearRatio);
  private final SwerveModule m_module1 = new SwerveModule(11, 21, driveGearRatio, azimuthGearRatio);
  private final SwerveModule m_module2 = new SwerveModule(12, 22, driveGearRatio, azimuthGearRatio);
  private final SwerveModule m_module3 = new SwerveModule(13, 23, driveGearRatio, azimuthGearRatio);

  private final Pigeon2 m_pigeon = new Pigeon2(6, "canivore");

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_module0Location, m_module1Location, m_module2Location, m_module3Location);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
          m_pigeon.getRotation2d(),
          new SwerveModulePosition[] {
            m_module0.getPosition(),
            m_module1.getPosition(),
            m_module2.getPosition(),
            m_module3.getPosition()
          });

  public Drivetrain() {
    m_pigeon.reset();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, m_pigeon.getRotation2d())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot),
                periodSeconds));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_module0.setDesiredState(swerveModuleStates[0]);
    m_module1.setDesiredState(swerveModuleStates[1]);
    m_module2.setDesiredState(swerveModuleStates[2]);
    m_module3.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        m_pigeon.getRotation2d(),
        new SwerveModulePosition[] {
          m_module0.getPosition(),
          m_module1.getPosition(),
          m_module2.getPosition(),
          m_module3.getPosition()
        });
  }
}
