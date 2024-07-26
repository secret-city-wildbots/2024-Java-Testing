// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class SysId extends SubsystemBase {
  // The motors on the left side of the drive.
    // Define constant for Canbus name on Holicanoli
    private static final String CANBUS_NAME = "rio";
    // Define the kraken motor on Holicanoli
    private final TalonFX m_leftMotor1 = new TalonFX(11, CANBUS_NAME);
    private final TalonFX m_leftMotor2 = new TalonFX(12, CANBUS_NAME);
    private final TalonFX m_rightMotor1 = new TalonFX(10, CANBUS_NAME);
    private final TalonFX m_rightMotor2 = new TalonFX(13, CANBUS_NAME);

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  // Create a new SysId routine for characterizing the drive.
  private final SysIdRoutine m_sysIdRoutineLeft1 =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motors.
              (Measure<Voltage> volts) -> {
                m_leftMotor1.setVoltage(volts.in(Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the left motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            m_leftMotor1.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(m_leftMotor1.getPosition().getValueAsDouble(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(m_leftMotor1.getVelocity().getValueAsDouble(), MetersPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("drive")
              this));
  //these next three sysid routines are the exact same as the first
  private final SysIdRoutine m_sysIdRoutineLeft2 =
      new SysIdRoutine(
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              (Measure<Voltage> volts) -> {
                m_leftMotor2.setVoltage(volts.in(Volts));
              },
              log -> {
                log.motor("drive")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            m_leftMotor2.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(m_leftMotor2.getPosition().getValueAsDouble(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(m_leftMotor2.getVelocity().getValueAsDouble(), MetersPerSecond));
              },
              this));
  private final SysIdRoutine m_sysIdRoutineRight1 =
      new SysIdRoutine(
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              (Measure<Voltage> volts) -> {
                m_rightMotor1.setVoltage(volts.in(Volts));
              },
              log -> {
                log.motor("drive")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            m_rightMotor1.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(m_rightMotor1.getPosition().getValueAsDouble(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(m_rightMotor1.getVelocity().getValueAsDouble(), MetersPerSecond));
              },
              this));
  private final SysIdRoutine m_sysIdRoutineRight2 =
      new SysIdRoutine(
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              (Measure<Voltage> volts) -> {
                m_rightMotor2.setVoltage(volts.in(Volts));
              },
              log -> {
                log.motor("drive")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            m_rightMotor2.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(m_rightMotor2.getPosition().getValueAsDouble(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(m_rightMotor2.getVelocity().getValueAsDouble(), MetersPerSecond));
              },
              this));

  /**
   * Returns a command that drives the robot with arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */

  /**
   * Returns a command that will execute a quasistatic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public void sysIdQuasistatic(SysIdRoutine.Direction direction) {
    m_sysIdRoutineLeft1.quasistatic(direction);
    m_sysIdRoutineLeft2.quasistatic(direction);
    m_sysIdRoutineRight1.quasistatic(direction);
    m_sysIdRoutineRight2.quasistatic(direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public void sysIdDynamic(SysIdRoutine.Direction direction) {
    m_sysIdRoutineLeft1.dynamic(direction);
    m_sysIdRoutineLeft2.dynamic(direction);
    m_sysIdRoutineRight1.dynamic(direction);
    m_sysIdRoutineRight2.dynamic(direction);
  }
}
