// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.ctre;

public class MagEncoderAbsoluteConfiguration {
  private final int id;
  private final double offset;

  public MagEncoderAbsoluteConfiguration(int id, double offset) {
      this.id = id;
      this.offset = offset;
  }

  public int getId() {
      return id;
  }

  public double getOffset() {
      return offset;
  }

  @Override
  public String toString() {
      return "MagEncoderConf{id=" + this.id + ", offset=" + this.offset + "}"; 
  }
}