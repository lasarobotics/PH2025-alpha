// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import org.lasarobotics.hardware.revrobotics.Spark;

public class ArmSubsystem {
  private Spark m_armMotor;
  private Spark m_rollerMotor;

  public static class Hardware {
    private Spark armMotor;
    private Spark rollerMotor;

    public Hardware(Spark armMotor, Spark rollerMotor) {
      this.armMotor = armMotor;
      this.rollerMotor = rollerMotor;
    }
  }
}
