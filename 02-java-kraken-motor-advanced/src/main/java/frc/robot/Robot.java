// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
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
  private final TalonFX kraken = new TalonFX(43, CANBUS_NAME);
  // Define krakenOut - This control mode will output a proportion of the supplied voltage which is supplied by the user.
  private final DutyCycleOut krakenOut = new DutyCycleOut(0);
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
    // krakenOut.Output - Proportion of supply voltage to apply in fractional units between -1 and +1
    // Remember the controller Left and Right joysticks (x & y axis) output a range from -1 to +1
    // Negate the value to align with physical movements (up is forward and down and backward)
    krakenOut.Output = -m_stick.getLeftY();

    // Enable / Disable FOC
    enableFOC = true;

    // setControl​(DutyCycleOut request) - This control mode will output a proportion of the supplied voltage which is supplied by the user.
    kraken.setControl(krakenOut.withEnableFOC(enableFOC));

    /*
     * Calculate the distance travelled based on the kraken motor values
     */

    // Get the current motor shaft rotations
    current_motor_rotations = kraken.getPosition().getValueAsDouble();
    // Calculate the change in motor shaft rotations. This will be + / - depending on which way the motor is turning
    delta_motor_rotations = current_motor_rotations - previous_motor_rotations;
    // Calculate the wheel shaft rotations
    delta_wheel_rotations = delta_motor_rotations / gear_ratio;
    // Calculate the distance travelled (inches)
    wheel_distance = wheel_distance + delta_wheel_rotations * 2 * Math.PI * wheel_radius;
    // Set previous_motor_rotations to current_motor_rotations for next loop cycle
    previous_motor_rotations = current_motor_rotations;

    // Log infomation to the network table
    /*
     * kraken.getVelocity() - Velocity of the device in mechanism rotations per second. This can be the velocity of
     * a remote sensor and is affected by the RotorToSensorRatio and SensorToMechanismRatio configs.
     * 
     * Default Settings
     *   - Minimum Value: -512.0
     *   - Maximum Value: 511.998046875
     *   - Default Value: 0
     *   - Units: rotations per second
     * 
     * Default Rates
     *   - CAN 2.0: 50.0 Hz
     *   - CAN FD: 100.0 Hz (TimeSynced with Pro)
     */

    /*
     * Since we have the kraken x60 setup to the default values, I am expecting to see the following:
     * 
     * Kraken x60 (FOC) RPM = 5800
     * Kraken x60 (FOC) RPS = 96.66
     * 
     * Kraken x60 (Trapezoidal) RPM = 6000
     * Kraken x60 (Trapezoidal) RPS = 100
     * 
     * I am expecting to see the graph peak at 96.66 RPS when we give full power to the motor for FOC
     * I am expecting to see the graph peak at 100 RPS when we give full power to the motor for Trapezoidal
     */

    krakenRotorVelocity.set(kraken.getVelocity().getValueAsDouble());


    /*
     * kraken.getPosition() - Position of the device in mechanism rotations. This can be the position of a
     * remote sensor and is affected by the RotorToSensorRatio and SensorToMechanismRatio configs, as well as
     * calls to setPosition.
     * 
     * Default Settings
     *   - Minimum Value: -16384.0
     *   - Maximum Value: 16383.9
     *   - Default Value: 0
     *   - Units: rotations
     * 
     * Default Rates
     *   - CAN 2.0: 50.0 Hz
     *   - CAN FD: 100.0 Hz (TimeSynced with Pro)
     */
    krakenRotorRotations.set(kraken.getPosition().getValueAsDouble());

    /* 
     * I am expecting the distance to go up and down depending on the direction of the motor.
     */
    wheelDistanceTravelled.set(wheel_distance);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
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