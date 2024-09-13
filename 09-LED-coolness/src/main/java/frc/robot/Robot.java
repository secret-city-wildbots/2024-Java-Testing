// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import frc.robot.SysId;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine; // Uncomment for SysID

public class Robot extends TimedRobot {
  public static enum MasterStates {
    STOWED,
    SHOOTING,
    AMP,
    CLIMBING,
    TRAP
  }

  public static MasterStates masterState = MasterStates.STOWED;

  private final XboxController m_driverController = new XboxController(0);
  private final XboxController m_manipController = new XboxController(1);
  private final Drivetrain m_swerve = new Drivetrain();
  private final Intake m_intake = new Intake(0.5, 0.5, 0.5);
  private final Shooter m_shooter = new Shooter(0.7, 0.576, 98);
  // private final SysId m_sysid = new SysId(); //Uncomment for sysID

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(6);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(6);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  @Override
  public void autonomousPeriodic() {
    driveWithJoystick(false);
    m_swerve.updateOdometry();
  }

  @Override
  public void teleopPeriodic() {
    /*
     * Only for running SysID
     * if (m_controller.getRightBumper()){
     * if (m_controller.getBButton()) {
     * m_sysid.sysIdQuasistatic(SysIdRoutine.Direction.kForward);
     * }
     * if (m_controller.getAButton()) {
     * m_sysid.sysIdQuasistatic(SysIdRoutine.Direction.kReverse);
     * }
     * if (m_controller.getYButton()) {
     * m_sysid.sysIdDynamic(SysIdRoutine.Direction.kForward);
     * }
     * if (m_controller.getXButton()) {
     * m_sysid.sysIdDynamic(SysIdRoutine.Direction.kReverse);
     * }
     * } else {
     */
    // Start by updating all sensor values
    getHighPrioritySensors();

    // Check for any drive updates and drive accordingly
    driveWithJoystick(true);
    Pose2d robotPosition = m_swerve.updateOdometry().getPoseMeters();

    // Check for state updates based on manip inputs
    updateMasterState();

    // Toggle intake if necessary
    m_intake.updateIntake(m_driverController, m_shooter.spunUp);

    // Automatically adjust wrist and shooter based off of master state and
    // controller inputs
    m_shooter.updateWrist(robotPosition);
    m_shooter.updateShooter(m_driverController.getRightTriggerAxis() > 0.2,
        m_driverController.getLeftTriggerAxis() > 0.7, robotPosition, m_intake.bbBroken);

    updateOutputs();
  }
  // }

  private void getHighPrioritySensors() {
    m_intake.updateSensors();
    m_shooter.updateSensors();
  }

  private void updateOutputs() {
    m_intake.updateOutputs();
    m_shooter.updateOutputs();
  }

  @Override
  public void testPeriodic() {
    // NOTE: Testing logging and seeing values on advantageScope
    m_swerve.advantageScope(m_driverController);
  }

  public void updateMasterState() {
    /*
     * Change master states to match these manip inputs:
     * Left Bumper: STOW
     * Right Bumper: AMP
     * Right Trigger: SHOOTING
     * Left Trigger & Start Button: CLIMBING
     */
    if (m_manipController.getLeftBumper()) {
      masterState = MasterStates.STOWED;
    } else if (masterState != MasterStates.CLIMBING && m_manipController.getRightBumper()) {
      masterState = MasterStates.AMP;
    } else if (masterState != MasterStates.CLIMBING && m_manipController.getRightTriggerAxis() > 0.7) {
      masterState = MasterStates.SHOOTING;
    } else if (m_manipController.getLeftTriggerAxis() > 0.7 && m_manipController.getStartButton()) {
      masterState = MasterStates.CLIMBING;
    }
  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(-m_driverController.getLeftY(), 0.08))
        * Drivetrain.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed = -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_driverController.getLeftX(), 0.08))
        * Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot = -m_rotLimiter.calculate(MathUtil.applyDeadband(m_driverController.getRightX(), 0.08))
        * Drivetrain.kMaxAngularSpeed;

    m_swerve.drive(xSpeed, -ySpeed,
        -rot, fieldRelative, getPeriod());
  }
}
