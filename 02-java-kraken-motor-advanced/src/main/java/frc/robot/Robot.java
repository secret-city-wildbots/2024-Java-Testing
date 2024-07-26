// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoublePublisher;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /*
   * Robot specific variables and constants
   */
  // Define constant for Canbus name on Holicanoli
  private static final String CANBUS_NAME = "rio";
  // Define the kraken motor on Holicanoli
  private final TalonFX kraken = new TalonFX(40, CANBUS_NAME);
  // Define krakenOut - This control mode will output a proportion of the supplied voltage which is supplied by the user.
  private final DutyCycleOut krakenOut = new DutyCycleOut(0);

  private final PIDController m_PIDController = new PIDController(0.096383, 0.0, 0.0);
  // Define enableFOC - This will contorl if the motor will use FOC mode or not
  private Boolean enableFOC = false;
  // Define the joystick (xbox controller)
  private XboxController m_stick;
  // Distance calculation variables needed for 1 wheel with our 2024 robot swerve specs
  private double wheel_distance; // Distance the wheel has travelled in inches
  private double current_motor_rotations; // This will be in rotations
  private double previous_motor_rotations; // This will be in rotations
  private double delta_motor_rotations; // This will be in rotations 
  private double delta_wheel_rotations; // This will be in rotations
  private final double wheel_radius = 2.5; // Radius in inches
  private final double gear_ratio = 7; // input / output
  private double rainbowStatus = 0;

  final AddressableLED m_led = new AddressableLED(9);
    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    final AddressableLEDBuffer m_ledBuffer  = new AddressableLEDBuffer(52);


  /*
   * Network table variables and constants
   */

  // Define a network table instance
  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  // Define a network table table within that instance
  private NetworkTable table = inst.getTable("datatable");
  // Define a network table table values to display
  private DoublePublisher krakenRotorVelocity = table.getDoubleTopic("krakenRotorVelocity").publish();
  private DoublePublisher krakenRotorRotations = table.getDoubleTopic("krakenRotorRotations").publish();
  private DoublePublisher wheelDistanceTravelled = table.getDoubleTopic("wheelDistanceTravelled").publish();

  /*
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setHSV(i, 240, 100, 100);
   }

    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();

    // Initialize a new TalonFXConfiguration Class
    var krakenConfiguration = new TalonFXConfiguration();

    // Initialize a new XboxController
    m_stick = new XboxController(0);

    /* User can optionally change the configs or leave it alone to perform a factory default */
    krakenConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    kraken.getConfigurator().apply(krakenConfiguration);
    kraken.setSafetyEnabled(true);

    // Reset motor and wheel position to 0.0
    kraken.setPosition(0.0);
    wheel_distance = 0.0;

    SmartDashboard.putData("Drive PID Controller", m_PIDController);
  }

  private void updateLEDRainbow() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the HSV values
      m_ledBuffer.setRGB(i, i*20 + (int)(rainbowStatus), 100, 100);
   }

    // Set the data
    m_led.setData(m_ledBuffer);

    rainbowStatus+=2;
    rainbowStatus%=360;
  }
  private void updateLEDLoading() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the HSV values
      m_ledBuffer.setRGB(i, 0, (i==(int)rainbowStatus) ? 255:0, (i==(int)rainbowStatus) ? 0:0);
   }

    // Set the data
    m_led.setData(m_ledBuffer);

    rainbowStatus+=0.5;
    rainbowStatus%=m_ledBuffer.getLength();
  }

  @Override
  public void robotPeriodic() {
    
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    updateLEDRainbow();
    // krakenOut.Output - Proportion of supply voltage to apply in fractional units between -1 and +1
    // Remember the controller Left and Right joysticks (x & y axis) output a range from -1 to +1
    // Negate the value to align with physical movements (up is forward and down and backward)
    SmartDashboard.putNumber("Drive PID Controller ACTUAL", (kraken.getRotorVelocity().getValueAsDouble()));
    SmartDashboard.putNumber("Drive PID Controller ACTUAL mathed", (kraken.getRotorVelocity().getValueAsDouble() / 7) * (2 * Math.PI * 0.0636));

    final double driveOutput = MathUtil.clamp(
      m_PIDController.calculate(
        (kraken.getRotorVelocity().getValueAsDouble() / 7) * (2 * Math.PI * 0.0636),
        -m_stick.getLeftY()
      ),
      -1.0, 
      1.0
    );

   kraken.set(driveOutput);

   // krakenOut.Output = driveOutput;
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    updateLEDLoading();
    // krakenOut.Output - Proportion of supply voltage to apply in fractional units between -1 and +1
    // Zero out controls in disabled mode so we aren't relying on the enable frame
    krakenOut.Output = 0;
    // setControl​(DutyCycleOut request) - This control mode will output a proportion of the supplied voltage which is supplied by the user.
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