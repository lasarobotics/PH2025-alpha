// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;
import org.lasarobotics.utils.PIDConstants;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase implements AutoCloseable {
  public static record Hardware(Spark armMotor, Spark rollerMotor) {}

  private Spark m_armMotor;
  private Spark m_rollerMotor;
  private SparkBaseConfig m_armMotorConfig;
  private SparkBaseConfig m_rollerMotorConfig;
  private final Current ARM_MOTOR_CURRENT_LIMIT = Constants.Arm.CURRENT_LIMIT;
  private final Current ROLLER_MOTOR_CURRENT_LIMIT = Constants.Arm.CURRENT_LIMIT;


  public IntakeSubsystem (Hardware armHardware, PIDConstants intakePID, Current driveCurrentLimit) {
    this.m_armMotor = armHardware.armMotor;
    this.m_rollerMotor = armHardware.rollerMotor;

    m_armMotorConfig = new SparkMaxConfig();
    m_rollerMotorConfig = new SparkMaxConfig();
  
    m_armMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
  
    m_armMotorConfig.closedLoop.pidf(
        intakePID.kP,
        intakePID.kI,
        intakePID.kD,
        intakePID.kF
    );  

    m_armMotorConfig.closedLoop.maxMotion.maxVelocity(1420);
    m_armMotorConfig.closedLoop.maxMotion.maxAcceleration(1420);
    m_rollerMotorConfig.closedLoop.maxMotion.maxVelocity(750);
    m_rollerMotorConfig.closedLoop.maxMotion.maxAcceleration(750);

    m_armMotorConfig.smartCurrentLimit((int)ARM_MOTOR_CURRENT_LIMIT.in(Units.Amps));
    m_rollerMotorConfig.smartCurrentLimit((int)ROLLER_MOTOR_CURRENT_LIMIT.in(Units.Amps));
    m_armMotor.setIdleMode(IdleMode.kBrake);

    m_armMotor.configure(m_armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    m_rollerMotor.configure(m_rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    m_armMotor.resetEncoder();
  }

  public static Hardware initializeHardware() {
    Hardware armHardware = new Hardware(
      new Spark(Constants.Arm.ARM_MOTOR_ID, MotorKind.NEO),
      new Spark(Constants.Arm.ROLLER_MOTOR_ID, MotorKind.NEO)
    );
    return armHardware;
  }

  /**
   * Stop the arm and rollers
   */
  private void stop() {
    m_rollerMotor.stopMotor();
    m_armMotor.stopMotor();
  }

  private void raiseArm() {
    m_armMotor.set(Constants.Arm.STOW_POS, ControlType.kMAXMotionPositionControl);
  }

  private void lowerArm() {
    m_armMotor.set(Constants.Arm.INTAKE_POS, ControlType.kMAXMotionPositionControl);
  }

  private void intakeAlgae() {
    m_rollerMotor.set(Constants.Arm.ROLLER_SPEED.in(Units.Percent)/1.5, ControlType.kDutyCycle);
  }

  private void holdPos() {
    m_armMotor.set(Constants.Arm.HOLD_ALGAE, ControlType.kMAXMotionPositionControl);
  }

  private void outtakeAlgae() {
    m_rollerMotor.set(-Constants.Arm.ROLLER_SPEED.in(Units.Percent), ControlType.kDutyCycle);
  }

  private void outtakeCoral() {
    m_rollerMotor.set(Constants.Arm.ROLLER_SPEED.in(Units.Percent)/2, ControlType.kDutyCycle);
  }

  public Command autoOuttakeCoralCommand() {
    return Commands.run(() -> outtakeCoral());
  }

  public Command holdPosCommand() {
    return startEnd(() -> holdPos(), () -> stop());
  }

  public Command outtakeCoralCommand() {
    return Commands.sequence(autoOuttakeCoralCommand().withTimeout(0.4), lowerArmCommand());
  }

  public Command lowerArmCommand() {
    return startEnd(() -> lowerArm(), () -> stop());
  }

  /**
   * Command to intake algae
   * @return Command to run the roller motor
   */
  public Command intakeAlgaeCommand() {
    return startEnd(() -> {
      intakeAlgae();
      lowerArm();
      }, () -> stop());
  }

  /**
   * Command to outtake algae
   * @return Command to run the roller motor in reverse
   */
  public Command outtakeAlgaeCommand() {
    return startEnd(() -> outtakeAlgae(), () -> stop());
  }

  public Command raiseArmCommand() {
    return startEnd(() -> raiseArm(), () -> stop());
  }

  /**
   * Command to stop the motors
   * @return Command to stop both of the arm motors
   */
  public Command stopCommand() {
    return runOnce(() -> stop());
  }

  @Override
  public void periodic() {
    //System.out.println(m_armMotor.getInputs().encoderPosition);
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
