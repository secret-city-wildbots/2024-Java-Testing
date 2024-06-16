// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String CANBUS_NAME = "rio";
  private final TalonFX kraken = new TalonFX(43, CANBUS_NAME);


  private final DutyCycleOut krakenOut = new DutyCycleOut(0);

  private Joystick m_stick;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    /* Configure the devices */
    var krakenConfiguration = new TalonFXConfiguration();

    m_stick = new Joystick(0);

    /* User can optionally change the configs or leave it alone to perform a factory default */
    krakenConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;


    kraken.getConfigurator().apply(krakenConfiguration);


  
  
    kraken.setSafetyEnabled(true);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    krakenOut.Output = m_stick.getX();
    kraken.setControl(krakenOut);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    /* Zero out controls so we aren't just relying on the enable frame */
    krakenOut.Output = 0;
    kraken.setControl(krakenOut);
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