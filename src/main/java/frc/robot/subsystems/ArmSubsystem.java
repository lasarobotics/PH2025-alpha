// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase implements AutoCloseable {
  public static record Hardware(Spark armMotor, Spark rollerMotor) {}

  private Spark m_armMotor;
  private Spark m_rollerMotor;

  public ArmSubsystem (Hardware armHardware) {
    this.m_armMotor = armHardware.armMotor;
    this.m_rollerMotor = armHardware.rollerMotor;
  }

  public static Hardware initializeHardware() {
    Hardware armHardware = new Hardware(
      new Spark(Constants.Arm.ARM_MOTOR_ID, MotorKind.NEO),
      new Spark(Constants.Arm.ROLLER_MOTOR_ID, MotorKind.NEO)
    );
    return armHardware;
  }

  /**
   * Stop climb
   */
  private void stop() {
    m_armMotor.stopMotor();
    m_rollerMotor.stopMotor();
  }

  private void raiseArm() {
    m_armMotor.set(Constants.Arm.ARM_SPEED.in(Units.Percent), ControlType.kDutyCycle);
  }

  private void lowerArm() {
    m_armMotor.set(-Constants.Arm.ARM_SPEED.in(Units.Percent), ControlType.kDutyCycle);
  }

  private void intake() {
    m_rollerMotor.set(Constants.Arm.ROLLER_SPEED.in(Units.Percent), ControlType.kDutyCycle);
  }

  private void outtake() {
    m_rollerMotor.set(-Constants.Arm.ROLLER_SPEED.in(Units.Percent), ControlType.kDutyCycle);
  }

  /**
   * Command to raise the arm
   * @return Command to run arm motor
   */
  public Command raiseArmCommand() {
    return startEnd(() -> raiseArm(), () -> stop());
  }

  /**
   * Command to lower the arm
   * @return Command to run arm motor in reverse
   */
  public Command lowerArmCommand() {
    return startEnd(() -> lowerArm(), () -> stop());
  }

  /**
   * Command to intake algae
   * @return Command to run the roller motor
   */
  public Command intakeCommand() {
    return startEnd(() -> intake(), () -> stop());
  }

  /**
   * Command to outtake algae
   * @return Command to run the roller motor in reverse
   */
  public Command outtakeCommand() {
    return startEnd(() -> outtake(), () -> stop());
  }

  /**
   * Command to stop the motors
   * @return Command to stop both of the arm motors
   */
  public Command stopCommand() {
    return runOnce(() -> stop());
  }

  /**
   * Method to close the motors
   * @return Closes both of the arm motors
   */
  @Override
  public void close() {
    m_armMotor.close();
    m_rollerMotor.close();
  }
}
