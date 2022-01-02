/*
  This sketch reads a raw Stream of RGB565 pixels
  from the Serial port and displays the frame on
  the window.

  Use with the Examples -> CameraCaptureRawBytes Arduino sketch.

  This example code is in the public domain.
*/

import processing.serial.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

Serial myPort;

// must match resolution used in the sketch
final int cameraWidth = 28;
final int cameraHeight = 28;
final int cameraBytesPerPixel = 1;
final int bytesPerFrame = cameraWidth * cameraHeight * cameraBytesPerPixel;

PImage myImage;
byte[] frameBuffer = new byte[bytesPerFrame];
int imgNumber = 001;

void setup()
{
  size(28, 28);

  // if you have only ONE serial port active  
  //myPort = new Serial(this, Serial.list()[0], 9600);          // if you have only ONE serial port active

  // if you know the serial port name
  //myPort = new Serial(this, "COM5", 9600);                    // Windows
  //myPort = new Serial(this, "/dev/ttyACM0", 9600);            // Linux
  myPort = new Serial(this, "/dev/cu.usbmodem14101", 9600);     // Mac

  // wait for full frame of bytes
  myPort.buffer(bytesPerFrame);  

  myImage = createImage(cameraWidth, cameraHeight, RGB);
}

void draw()
{
  image(myImage, 0, 0);
}

void serialEvent(Serial myPort) {
  // read the saw bytes in
  myPort.readBytes(frameBuffer);

  // access raw bytes via byte buffer
  ByteBuffer bb = ByteBuffer.wrap(frameBuffer);
  bb.order(ByteOrder.BIG_ENDIAN);

  int i = 0;

  while (bb.hasRemaining()) {
    // read 16-bit pixel
    //short p = bb.getShort();
    // read 8-bit pixel
    byte p = bb.get();

    // convert RGB565 to RGB 24-bit
    //int r = ((p >> 11) & 0x1f) << 3;
    //int g = ((p >> 5) & 0x3f) << 2;
    //int b = ((p >> 0) & 0x1f) << 3;

    // set pixel color
    //myImage .pixels[i++] = color(r, g, b);
    myImage.pixels[i++] = color((int) (p & 0xFF));
  }
  
 myImage .updatePixels();
}

void keyPressed()
{
  if (key == 'c')
  {
    String filename = "capacitor/";
    filename = filename + nf(imgNumber++, 3) + ".png";
    save(filename);
  }
}
