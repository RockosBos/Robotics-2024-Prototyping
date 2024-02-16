// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LED extends SubsystemBase {
  /** Creates a new LED. */
  AddressableLED led;
  AddressableLEDBuffer ledBuffer;

  public LED() {
    led = new AddressableLED(Constants.ledPort);

    ledBuffer = new AddressableLEDBuffer(Constants.ledLength);
    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
    led.start();
  }

  @Override
  public void periodic() {
    //ledBuffer.setRGB(0, 255, 0, 0);
    for(int i = 0; i < ledBuffer.getLength(); i++){
      ledBuffer.setRGB(i, 255, 0 , 0);
    }
    //led.setData(ledBuffer);
  }
}
