/*
  This sketch reads a raw Stream of RGB565 or grayscale pixels
  from the Serial port and displays the frame on
  the window.

  Use with the Examples -> CameraCaptureRawBytes Arduino sketch.
  
  Captured image can be saved into a file in <dstFolder>/<imgNumber>.png
  by pressing 'c'.

  This example code is in the public domain.
  
  Original code in Arduino_OV767X 'CameraVisualizerRawBytes' example sketch
  Edited by : adamiaonr@gmail.com
*/

import processing.serial.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

Serial myPort;

// must match resolution used in the sketch
static final int cameraWidth = 176;
static final int cameraHeight = 144;
static final int cameraBytesPerPixel = 1;
static final int bytesPerFrame = cameraWidth * cameraHeight * cameraBytesPerPixel;
static final int gridWidth = 36;
static final int gridHeight = 36;
// overlays a grid with cells of size gridWidth x gridHeight 
static final boolean drawGrid = false;

static final String serialPortName = "/dev/cu.usbmodem14101";
static final String dstFolder = "img/";

PImage myImage;
byte[] frameBuffer = new byte[bytesPerFrame];
int imgNumber = 1;

void settings()
{
  // by default, Processing does not allow size() to be called with variables in setup()
  // we can nevertheless do this within settings()
  size(cameraWidth, cameraHeight);
}

void setup()
{
  // if you have only ONE serial port active  
  //myPort = new Serial(this, Serial.list()[0], 9600);          // if you have only ONE serial port active

  // if you know the serial port name
  //myPort = new Serial(this, "COM5", 9600);                    // Windows
  //myPort = new Serial(this, "/dev/ttyACM0", 9600);            // Linux
  myPort = new Serial(this, serialPortName, 9600);              // Mac

  // wait for full frame of bytes
  myPort.buffer(bytesPerFrame);  

  myImage = createImage(cameraWidth, cameraHeight, RGB);
}

void draw()
{
  image(myImage, 0, 0);
  
  // overlays a grid with cells of size gridWidth x gridHeight 
  if (drawGrid)
  {
    for (int c = gridWidth; c < cameraWidth; c += gridWidth)
    {
      line(c, 0, c, gridHeight);    
    }
    
    for (int r = gridHeight; r < cameraHeight; r += gridHeight)
    {
      line(0, r, gridWidth, r);
    }
  }
}

void serialEvent(Serial myPort) 
{
  // read the saw bytes in
  myPort.readBytes(frameBuffer);

  // access raw bytes via byte buffer
  ByteBuffer bb = ByteBuffer.wrap(frameBuffer);
  bb.order(ByteOrder.BIG_ENDIAN);

  int i = 0;

  while (bb.hasRemaining()) {
    
    if (cameraBytesPerPixel == 1)
    {
      // read 8-bit pixel
      byte p = bb.get();
      // set pixel color
      myImage.pixels[i++] = color((int) (p & 0xFF));
    }
    else
    {
      // read 16-bit pixel
      short p = bb.getShort();
      // convert RGB565 to RGB 24-bit
      int r = ((p >> 11) & 0x1f) << 3;
      int g = ((p >> 5) & 0x3f) << 2;
      int b = ((p >> 0) & 0x1f) << 3;
      // set pixel color
      myImage .pixels[i++] = color(r, g, b);
    }
  }
  
  myImage.updatePixels();
}

void keyPressed()
{
  if (key == 'c')
  {
    String filename = dstFolder + nf(imgNumber++, 3) + ".png";
    save(filename);
  }
}
