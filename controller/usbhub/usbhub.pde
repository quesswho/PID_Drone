import java.nio.*;
import net.java.games.input.*;
import org.gamecontrolplus.*;
import org.gamecontrolplus.gui.*;

import processing.serial.*;

ControlIO control;
ControlDevice device;

static byte[] packet;

int mx, my, rot, alt; // MoveX, MoveY, Rotate, Altitude
int lmx, lmy, lrot, lalt; // Previous data
boolean blink = false;
boolean lblink = false;

final int rate = 38400;

Serial serial;

void setup() {
  size(600, 400);
  
  packet = new byte[10];
  // Initialize Controller
  control = ControlIO.getInstance(this);
  device = control.filter(GCP.GAMEPAD).getMatchedDevice("gamepad");
  if(device == null) {
    println("Device was not found!");
    System.exit(-1);
  }
  
  // Initialize Serial
  String portName = Serial.list()[1];
  serial = new Serial(this, portName, rate);
  println("Using " + portName + " as serial device.");
}

int floatmap(final float f) {
  return (int)(f * 512.0f);
}

void read_user_input() {
  mx = floatmap(device.getSlider("MX").getValue());
  my = floatmap(device.getSlider("MY").getValue());
  rot = floatmap(device.getSlider("ROT").getValue());
  alt = floatmap(-device.getSlider("ALT").getValue()); // negated so right trigger produces positive values.
  blink = device.getButton("BLINK").pressed();
}

void deadzone_fix() {
   if(abs(mx) < 13) {
      mx = 0;
    }
    if(abs(my) < 13) {
      my = 0;
    }
    if(abs(rot) < 17) {
      rot = 0;
    }
}

boolean build_packet() {
  byte mask = 0;
  if(lmx != mx) {
    lmx = mx;
    mask |= 1 << 0;
    packet[1] = (byte)((mx >> 8) & 0xff);
    packet[2] = (byte)((mx >> 0) & 0xff);
  }
  if(lmy != my) {
    lmy = my;
    mask |= 1 << 1;
    packet[3] = (byte)((my >> 8) & 0xff);
    packet[4] = (byte)((my >> 0) & 0xff);
  }
  if(lrot != rot) {
    lrot = rot;
    mask |= 1 << 2;
    packet[5] = (byte)((rot >> 8) & 0xff);
    packet[6] = (byte)((rot >> 0) & 0xff);
  }
  if(lalt != alt) {
    lalt = alt;
    mask |= 1 << 3;
    packet[7] = (byte)((alt >> 8) & 0xff);
    packet[8] = (byte)((alt >> 0) & 0xff);
  }
  if(lblink != blink) {
    lblink = blink;
    mask |= 1 << 4;
    packet[9] = (byte)(blink ? 1 : 0);
  }
  if(mask == 0) return false;
  mask |= 1 << 7;
  packet[0] = mask;
  return true;
}

void draw() {
  read_user_input();
  deadzone_fix();
  if(build_packet()) {
    serial.write(packet);
  }
  
  // Debug
  while (serial.available() > 0) {
    print(serial.readChar());
  }
}
