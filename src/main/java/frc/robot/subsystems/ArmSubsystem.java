// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import org.lasarobotics.hardware.revrobotics.Spark;
import com.revrobotics.spark.SparkBase.ControlType;

import frc.robot.Constants;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem {
  private Spark m_armMotor;
  private Spark m_rollerMotor;
  private Double m_armSpeed;
  private Double m_rollerSpeed;

  public static class Hardware {
    private Spark armMotor;
    private Spark rollerMotor;

    public Hardware(Spark armMotor, Spark rollerMotor) {
      this.armMotor = armMotor;
      this.rollerMotor = rollerMotor;
    }

    public static Hardware initializeHardware() {
      Hardware armHardware = new Hardware(
        new Spark(Constants.Arm.ARM_MOTOR_ID, MotorKind.NEO),
        new Spark(Constants.Arm.ROLLER_MOTOR_ID, MotorKind.NEO)
      );
      return armHardware;
    }
  }

  public ArmSubsystem (Hardware armHardware, Double armSpeed, Double rollerSpeed) {
    this.m_armMotor = armHardware.armMotor;
    this.m_rollerMotor = armHardware.rollerMotor;
    this.m_armSpeed = armSpeed;
    this.m_rollerSpeed = rollerSpeed;
  }

  private void raiseArm() {
    m_armMotor.set(m_armSpeed.in(Units.Percent), ControlType.kDutyCycle);
  }

  private void lowerArm() {
    m_armMotor.set(-m_armSpeed.in(Units.Percent), ControlType.kDutyCycle);
  }

  private void intake() {
    m_rollerMotor.set(m_rollerSpeed.in(Units.Percent), ControlType.kDutyCycle);
  }

  private void outtake() {
    m_rollerMotor.set(-m_rollerSpeed.in(Units.Percent), ControlType.kDutyCycle);
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
}
