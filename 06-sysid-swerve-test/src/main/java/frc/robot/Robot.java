// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.SysId;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Robot extends TimedRobot {
  private final XboxController m_controller = new XboxController(0);
  private final Drivetrain m_swerve = new Drivetrain();
  private final SysId m_sysid = new SysId();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(6);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(6);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  //is baby mode active
  private boolean babyModeActive = false;

  @Override
  public void autonomousPeriodic() {
    driveWithJoystick(false);
    m_swerve.updateOdometry();
  }

  @Override
  public void teleopPeriodic() {
    if (m_controller.getRightBumper() == true) {
      if (m_controller.getBButton()) {
        m_sysid.sysIdQuasistatic(SysIdRoutine.Direction.kForward);
      }
      if (m_controller.getAButton()) {
        m_sysid.sysIdQuasistatic(SysIdRoutine.Direction.kReverse);
      }
      if (m_controller.getYButton()) {
        m_sysid.sysIdDynamic(SysIdRoutine.Direction.kForward);
      }
      if (m_controller.getXButton()) {
        m_sysid.sysIdDynamic(SysIdRoutine.Direction.kReverse);
      }
    } else {
      checkForBabyToggle();
      driveWithJoystick(true);
    }
  }

  @Override
  public void testPeriodic() {
    // NOTE: Testing logging and seeing values on advantageScope
    m_swerve.advantageScope(m_controller);
  }

  private void checkForBabyToggle() {
    if (m_controller.getBButtonPressed()) {
      babyModeActive = !babyModeActive;
    }
  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(-m_controller.getLeftY(), 0.08))
            * Drivetrain.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.08))
            * Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot =
        -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.08))
            * Drivetrain.kMaxAngularSpeed;

    m_swerve.drive(xSpeed * ((babyModeActive) ? 0.2:1), -ySpeed * ((babyModeActive) ? 0.2:1), -rot * ((babyModeActive) ? 0.5:1), fieldRelative, getPeriod());
  }
}
