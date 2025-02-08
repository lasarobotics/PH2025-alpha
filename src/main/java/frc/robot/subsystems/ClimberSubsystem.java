package frc.robot.subsystems;

import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ClimberSubsystem extends SubsystemBase implements AutoCloseable {
  public static record Hardware(Spark climberMotor) {}

  private Spark m_climberMotor;

  /**
   * Create an instance of Climber Subsystem
   * <p>
   * NOTE: ONLY ONE INSTANCE SHOULD EXIST AT ANY TIME!
   * <p>
   * 
   * @param climbHardware Hardware devices required by climb
   */
  public ClimberSubsystem(Hardware climberHardware) {
    this.m_climberMotor = climberHardware.climberMotor;
  }

  /**
   * Initialize hardware devices for amp subsystem
   * 
   * @return hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    Hardware climbHardware = new Hardware(
      new Spark(Constants.Climber.CLIMB_MOTOR_ID, MotorKind.NEO)
    );
    return climbHardware;
  }

  /**
   * Stop climb
   */
  private void stop() {
    m_climberMotor.stopMotor();
  }

  /**
   * Runs the climb motor to raise arm
   */
  private void raiseClimber() {
    m_climberMotor.set(Constants.Climber.CLIMBER_SPEED.in(Units.Percent), ControlType.kDutyCycle);
  }

  /**
   * Runs the climb motor to lower arm
   */
  private void lowerClimber() {
    m_climberMotor.set(-Constants.Climber.CLIMBER_SPEED.in(Units.Percent), ControlType.kDutyCycle);
  }

  /**
   * Command to raise the climber hook
   * @return Command to run climber motor
   */
  public Command raiseClimbCommand() {
    return startEnd(() -> raiseClimber(), () -> stop());
  }

  /**
   * Command to lower the climber hook
   * @return Command to run climber motor in reverse
   */
  public Command lowerClimbCommand() {
    return startEnd(() -> lowerClimber(), () -> stop());
  }
  
  /**
   * Command to stop the motor
   * @return Command to stop the climber motor
   */
  public Command stopCommand() {
    return runOnce(() -> stop());
  }

  /**
   * Method to close the motor
   * @return Closes the arm motor
   */
  @Override
  public void close() {
    m_climberMotor.close();
  }
}