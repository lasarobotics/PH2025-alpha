// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



import org.lasarobotics.hardware.revrobotics.Spark;
public class ClimbSubsystem {

  private Spark m_climbMotor;


  public static class Hardware {
    private Spark climbMotor;

     public Hardware(Spark climbMotor) {
    this.climbMotor = climbMotor;
    }
  }

}
