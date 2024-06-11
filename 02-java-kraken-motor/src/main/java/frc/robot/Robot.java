// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String CANBUS_NAME = "rio";
  private final TalonFX leftLeader = new TalonFX(43, CANBUS_NAME);


  private final DutyCycleOut leftOut = new DutyCycleOut(0);

  private final XboxController joystick = new XboxController(0);

  private int printCount = 0;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    /* Configure the devices */
    var leftConfiguration = new TalonFXConfiguration();


    /* User can optionally change the configs or leave it alone to perform a factory default */
    leftConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;


    leftLeader.getConfigurator().apply(leftConfiguration);


  
  
    leftLeader.setSafetyEnabled(true);
  

    /* Currently in simulation, we do not support FOC, so disable it while simulating */
    if (Utils.isSimulation()){
      leftOut.EnableFOC = false;
      
    }
  }

  @Override
  public void robotPeriodic() {
    if (++printCount >= 10) {
      printCount = 0;
      System.out.println("Left out: " + leftLeader.get());
      System.out.println("Left Pos: " + leftLeader.getPosition());
    }
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    /* Get forward and rotational throttle from joystick */
    /* invert the joystick Y because forward Y is negative */
    double fwd = -joystick.getLeftY();
    double rot = joystick.getRightX();
    /* Set output to control frames */
    leftOut.Output = fwd + rot;

    /* And set them to the motors */
    if (!joystick.getAButton()) {
      leftLeader.setControl(leftOut);

    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    /* Zero out controls so we aren't just relying on the enable frame */
    leftOut.Output = 0;
    leftLeader.setControl(leftOut);
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}