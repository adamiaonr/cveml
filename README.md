# cveml

This repository contains a set of projects I've developed while taking the [Computer Vision with Embedded Machine Learning](https://www.coursera.org/learn/computer-vision-with-embedded-machine-learning) course, taught by Shawn Hymel and supported by the [Edge Impulse](https://www.edgeimpulse.com/) online tool.

## Overview

The results can be checked in [this page](https://adamiaonr.github.io/cveml/).

## Hardware Setup

I've used the following setup:

* 1 Arduino Nano 33 BLE Sense
* 1 OV7670 CMOS camera (e.g., like [this](https://www.openhacks.com/uploadsproductos/ov7670_cmos_camera_module_revc_ds.pdf))
* 1 small breadboard (e.g., [this](https://www.bananarobotics.com/shop/image/cache/data/sku/BR/0/1/0/1/9/BR010198-Small-400-Point-White-Breadboard/top-600x600.JPG))
* 18 M/F jumper wires, 10 cm long (e.g., [this](https://www.ptrobotics.com/jumper-wires/6484-premium-female-male-jumper-wires-100mm-pack-of-40.html))
* 1 'smartphone' tripod (e.g., [this](https://www.amazon.es/gp/product/B01K1VO0LW/ref=ppx_yo_dt_b_asin_title_o00_s00?ie=UTF8&psc=1)), used to hold the Arduino + Camera setup in place, making it easier to run live inference experiments

The images below show how my setup looks like.

![](docs/assets/images/setup-collage.jpg?raw=true)

Notice that I've also taped a blank A4 sheet to the top of my desk, which served as a background for both data captures and live inference tests. 

It is also important to ensure consistent lighting conditions between your training / testing data and live inference (e.g., using the light of a lamp, pointing directly at the blank sheet of paper). Either that, or you must train your classifier with data captured with different lighting conditions, so that to make your classifier 'insensitive' to lighting.

### Connections between Arduino Nano 33 BLE Sense and OV7670 camera

The connections between my particular OV7670 camera module and Arduino Nano 33 BLE Sense are shown in the table below. These connections can be found in the examples of the Arduino OV767X library, e.g. in the file `libraries/Arduino_OV767X/examples/CameraCapture/CameraCapture.ino` in the repository).

| Arduino Nano 33 BLE Sense pin | OV7670 CMOs camera pin | OV7670 CMOs camera pin (alt. name) |
|---|---|---|
| +3V3 | 3.3V | - |
| GND | DGND | GND |
| A5 | SDA | SIOC |
| A4 | SCL | SIOD |
| D8 | VS | VSYNC |
| A1 | HS | HREF |
| A0 | PLK | PCLK |
| D9 | XLK | XCLK |
| D4 | D7 | - |
| D6 | D6 | - |
| D5 | D5 | - |
| D3 | D4 | - |
| D2 | D3 | - |
| RX | D2 | - |
| TX | D1 | - |
| D10 | D0 | - |
| GND | PDWN | - |
| +3V3 | RET | - |

## Usage

### Projects

Each project in the `projects/` folder is designed to be deployed in an Arduino Nano 33 BLE Sense as a .zip library using the Arduino IDE.

To deploy each project, follow these steps (this assumes a hardware setup as described above):

1. `cd` to the project directory
2. Run `make` on the project directory : this will generate a .zip Arduino library, which can then be added to the Arduino IDE
4. Open the Arduino IDE, and add the resulting .zip file, using the `Sketch > Include Library > Add .ZIP Library` feature
4. Open the sketch via the `File > Examples > <project-name> > nano_ble33_sense_camera` menu
5. After loading the sketch, deploy via the `Sketch > Upload` menu

**Notes:** 

* Ensure the Arduino Nano 33 BLE Sense and the correct serial port are selected in the `Tools` menu. For further details, refer to the [Arduino documentation](https://docs.arduino.cc/hardware/nano-33-ble-sense).
* In order for this to work, you'll need to update the OV767X library with an extra method : `isGrayscale()`. You can use the library version that ships in this Github repository in the `libraries/` folder.

### Data capture

I've captured data for my datasets in `datasets/` directly with the Arduino + OV7670 setup : this way, I've ensured that the data used for training my image classifiers is captured in the same way as in the live inference tests.

I've used the [Processing](https://processing.org/download) desktop application (v4.0b2) collect the image data captured by the Arduino. Processing allows you to run Java 'sketches' that connect to the serial port (similarly to the Arduino IDE), read the image data captured and sent by the Arduino over the serial port, allowing you to display, transform and save the images in a typical format such as `.png` or `.jpg`.

I've included two sketches in the repository that can be used in conjunction to get you started with your data collection efforts, specifically:

* `libraries/Arduino_OV767X/examples/CameraCapture.ino` : runs continuously on the Arduino, performing the following operations :  

 1. Captures an image in QCIF and grayscale format (176 x 144 px, only the 'luminance' byte of the YUV422 format)
 2. Crops the image at the center to a custom size, using the same routines as in the live inference sketches
 3. Sends the cropped image over the serial port

* `libraries/Arduino_OV767X/extras/CameraVisualizerRawBytes.pde` : 

 1. Reads an image from the serial port. Image size and format must be specified in the sketch, using the variables below.
 2. Displays the image in a pop-up window
 3. Saves the image in `.png` format in the `dstFolder` folder

| Variable | Descr. |
|---|---|
| `cameraWidth`   | image width (px) as sent by Arduino |
| `cameraHeight ` | image height (px) |
| `cameraBytesPerPixel` | number of byte per pixel (e.g., 2 byte for RGB565, 1 byte for grayscale) |
| `dstFolder` | folder where images are saved|
 