// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.Pigeon2;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  // Simulation position
  private Pose3d position;
  private double translation3D_X = 20.0;
  private double translation3D_Y = 80.0;
  private double translation3D_Z = 0.0;
  private double rotation3D_roll = 0.0;
  private double rotation3D_pitch = 0.0;
  private double rotation3D_yaw = 0.0;

  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable table = inst.getTable("datatable");
  private StructPublisher<Pose3d> myPose = table.getStructTopic("myPose", Pose3d.struct).publish();
  
  public static final double kMaxSpeed = 5.8; // meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final Translation2d m_module0Location = new Translation2d(0.254, -0.311);
  private final Translation2d m_module1Location = new Translation2d(0.254, 0.311);
  private final Translation2d m_module2Location = new Translation2d(-0.254, 0.311);
  private final Translation2d m_module3Location = new Translation2d(-0.254, -0.311);

  public static final double driveGearRatio = 7;
  public static final double azimuthGearRatio = 20;

  // NOTE: setup to be used with Holicanoli uncomment line 52 to use with the real robot.
  // private final SwerveModule m_module0 = new SwerveModule(10, 43, driveGearRatio, azimuthGearRatio);
  private final SwerveModule m_module0 = new SwerveModule(10, 20, driveGearRatio, azimuthGearRatio);
  private final SwerveModule m_module1 = new SwerveModule(11, 21, driveGearRatio, azimuthGearRatio);
  private final SwerveModule m_module2 = new SwerveModule(12, 22, driveGearRatio, azimuthGearRatio);
  private final SwerveModule m_module3 = new SwerveModule(13, 23, driveGearRatio, azimuthGearRatio);

  private final Pigeon2 m_pigeon = new Pigeon2(6, "canivore");

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_module0Location, m_module1Location, m_module2Location, m_module3Location);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
        m_kinematics,m_pigeon.getRotation2d(),
        new SwerveModulePosition[] {
          m_module0.getPosition(),
          m_module1.getPosition(),
          m_module2.getPosition(),
          m_module3.getPosition()
        }
      );

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
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds)
  {
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
      ChassisSpeeds.discretize(
        fieldRelative ? 
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_pigeon.getRotation2d())
        : new ChassisSpeeds(xSpeed, ySpeed, rot), periodSeconds
      )
    );

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_module0.setDesiredState(swerveModuleStates[0]);
    m_module1.setDesiredState(swerveModuleStates[1]);
    m_module2.setDesiredState(swerveModuleStates[2]);
    m_module3.setDesiredState(swerveModuleStates[3]);

    // Build loggingState so advantageScope can see the values using the desired module states
    double loggingState[] = {
      swerveModuleStates[1].angle.getDegrees(), // FL Swerve Module Rotation (degrees)
      swerveModuleStates[1].speedMetersPerSecond, // FL Swerve Drive Speed (m/s)
      swerveModuleStates[0].angle.getDegrees(), // FR Swerve Module Rotation (degrees)
      swerveModuleStates[0].speedMetersPerSecond, // FR Swerve Drive Speed (m/s)
      swerveModuleStates[2].angle.getDegrees(), // BL Swerve Module Rotation (degrees)
      swerveModuleStates[2].speedMetersPerSecond, // BL Swerve Drive Speed (m/s)
      swerveModuleStates[3].angle.getDegrees(), // BR Swerve Module Rotation (degrees)
      swerveModuleStates[3].speedMetersPerSecond, // BR Swerve Drive Speed (m/s)
    };

    // The below code should log the actual values for the swerve modules
    // double loggingState[] = {
    //   m_module1.getState().angle.getDegrees(), // FL Swerve Module Rotation (degrees)
    //   m_module1.getState().speedMetersPerSecond, // FL Swerve Drive Speed (m/s)
    //   m_module0.getState().angle.getDegrees(), // FR Swerve Module Rotation (degrees)
    //   m_module0.getState().speedMetersPerSecond, // FR Swerve Drive Speed (m/s)
    //   m_module2.getState().angle.getDegrees(), // BL Swerve Module Rotation (degrees)
    //   m_module2.getState().speedMetersPerSecond, // BL Swerve Drive Speed (m/s)
    //   m_module3.getState().angle.getDegrees(), // BR Swerve Module Rotation (degrees)
    //   m_module3.getState().speedMetersPerSecond, // BR Swerve Drive Speed (m/s)
    // };

    // Send loggingState to the SmartDashboard to be seen in advantageScope
    SmartDashboard.putNumberArray("SwerveModuleStates", loggingState);
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

  public void advantageScope(XboxController controller) {
    // NOTE: Testing logging and seeing values on advantageScope
    double maxSpeed = 6.0; // m/s
    double maxRotation = 2 * Math.PI; // rad/s

    double xSpeedInput = -MathUtil.applyDeadband(controller.getLeftY(), 0.08);
    double ySpeedInput = -MathUtil.applyDeadband(controller.getLeftX(), 0.08);
    double rotSpeedInput = -MathUtil.applyDeadband(controller.getRawAxis(2), 0.08);

    var swerveModuleStates = m_kinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeedInput * maxSpeed,ySpeedInput * maxSpeed, rotSpeedInput * maxRotation));

    m_module0.setDesiredState(swerveModuleStates[0]);
    m_module1.setDesiredState(swerveModuleStates[1]);
    m_module2.setDesiredState(swerveModuleStates[2]);
    m_module3.setDesiredState(swerveModuleStates[3]);

    // double loggingState[] = {
    //   swerveModuleStates[1].angle.getDegrees(), // FL Swerve Module Rotation (degrees)
    //   swerveModuleStates[1].speedMetersPerSecond, // FL Swerve Drive Speed (m/s)
    //   swerveModuleStates[0].angle.getDegrees(), // FR Swerve Module Rotation (degrees)
    //   swerveModuleStates[0].speedMetersPerSecond, // FR Swerve Drive Speed (m/s)
    //   swerveModuleStates[2].angle.getDegrees(), // BL Swerve Module Rotation (degrees)
    //   swerveModuleStates[2].speedMetersPerSecond, // BL Swerve Drive Speed (m/s)
    //   swerveModuleStates[3].angle.getDegrees(), // BR Swerve Module Rotation (degrees)
    //   swerveModuleStates[3].speedMetersPerSecond, // BR Swerve Drive Speed (m/s)
    // };

    double loggingState[] = {
      m_module1.getState().angle.getDegrees(), // FL Swerve Module Rotation (degrees)
      m_module1.getState().speedMetersPerSecond, // FL Swerve Drive Speed (m/s)
      m_module0.getState().angle.getDegrees(), // FR Swerve Module Rotation (degrees)
      m_module0.getState().speedMetersPerSecond, // FR Swerve Drive Speed (m/s)
      m_module2.getState().angle.getDegrees(), // BL Swerve Module Rotation (degrees)
      m_module2.getState().speedMetersPerSecond, // BL Swerve Drive Speed (m/s)
      m_module3.getState().angle.getDegrees(), // BR Swerve Module Rotation (degrees)
      m_module3.getState().speedMetersPerSecond, // BR Swerve Drive Speed (m/s)
    };

    if (controller.getYButtonPressed()) {
      System.out.println("I am here");
      System.out.println(translation3D_Y);
      //move forward 10 inches
      translation3D_Y += 10.0;
    }
    if (controller.getAButtonPressed()) {
      System.out.println("testing A");
      //move back 10 inches
      translation3D_Y -= 10.0;
    }

    if (controller.getAButtonPressed()) {
      //move forward 10 inches
      translation3D_X += 10.0;
    }
    if (controller.getBButtonPressed()) {
      //move forward 10 inches
      translation3D_X += 10.0;
    }


    position = new Pose3d(
      new Translation3d(translation3D_X,translation3D_Y,translation3D_Z),
      new Rotation3d(rotation3D_roll,rotation3D_pitch,rotation3D_yaw)
    );

    myPose.set(position);
    SmartDashboard.putNumberArray("SwerveModuleStates", loggingState);
  }
}
