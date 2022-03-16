import net.java.games.input.*;
import org.gamecontrolplus.*;
import org.gamecontrolplus.gui.*;

import processing.serial.*;

ControlIO control;
ControlDevice device;

float mx, my, rot, alt; // MoveX, MoveY, Rotate, Altitude
boolean blink = false;
void setup() {
  size(600, 400);
  
  control = ControlIO.getInstance(this);
  device = control.filter(GCP.GAMEPAD).getMatchedDevice("gamepad");
  //device = control.filter(GCP.STICK).getDevice("DS3 Compatible HID Device");
  if(device == null) {
    println("Device was not found!");
    System.exit(-1);
  }
}

void read_user_input() {
  mx = device.getSlider("MX").getValue();
  my = device.getSlider("MY").getValue();
  rot = device.getSlider("ROT").getValue();
  alt = -device.getSlider("ALT").getValue(); // Feels more natural with the right trigger producing positive values.
  
  println(alt);
}

void draw() {
  read_user_input();
}
