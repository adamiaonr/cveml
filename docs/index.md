# CVEML : Computer Vision with Embedded Machine Learning

This page showcases my projects while taking the Coursera [Computer Vision with Embedded Machine Learning](https://www.coursera.org/learn/computer-vision-with-embedded-machine-learning) course, taught by Shawn Hymmel and supported by the [Edge Impulse](https://www.edgeimpulse.com/) online tool.

### Why should you care?

Is this another standard 'hand-in' from a course project? Not quite:

* Instead of using the recommended hardware platforms for the course as of November 2021 (i.e., OpenMV Camera or Raspberry Pi 4 with PiCamera), I've used [an Arduino Nano BLE 33 Sense](https://docs.arduino.cc/hardware/nano-33-ble-sense) connected to an [OV7670 CMOS camera](https://www.openhacks.com/uploadsproductos/ov7670_cmos_camera_module_revc_ds.pdf). 
* This work shows how to create a data capture and live inference setup with the Arduino + OV7670 camera, and also how to integrate the TensorFlow Lite models trained in Edge Impulse in Arduino.
* I've followed the proposed 'electronic component' classification task, but decided to make it harder by using 4 to 5 different components per class. E.g., I've built the LED dataset with photos from 4 different LEDs, the capacitor dataset with 5 different capacitors, etc.
* Instead of simply following the tutorial script, I've experimented with different model types and hyperparameter sets, comparing the live performance in the chosen classification task along two metrics: (i) accuracy and (ii) sensitivity to the position of the object in the frame. The results can be interesting to you, and are shown below.

## Index

## Overview

### Classification task

### Architecture

### Datasets & augmentation techniques

### Implementation details

## Usage

## Results


### Models and hyperparameters

I've recorded the performance of multiple Neural Network models, changing hyperparameters and using different versions of the electronic components base dataset published [here](https://github.com/adamiaonr/cveml/tree/main/datasets/electronic-parts-larger).

The complete list of models and hyperparameter sets I've tried out is listed below, along with the estimated inference time (using a Cortex M4F 80 MHz CPU as reference), RAM and flash usage.

| **id** | **model description**                                                                                                                                                      | **model descr. short**        | **dataset size (per class)** | **train epochs** |
|--------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------|------------------------------|------------------|
| 0      | "2 layers, 64 neuron each"                                                                                                                                                 | dnn-base                      | 50                           | 200              |
| 1      | "2 layers, 64 neuron each"                                                                                                                                                 | dnn-base                      | 2000                         | 200              |
| 2      | "2 layers, 64 neuron each"                                                                                                                                                 | dnn-base                      | 3500                         | 100              |
| 3      | "2 layers, 128 neuron each"                                                                                                                                                | dnn-2x-neuron                 | 2000                         | 200              |
| 4      | "2 layers, 128 neuron each"                                                                                                                                                | dnn-2x-neuron                 | 3500                         | 100              |
| 5      | "3 layers, 64 neuron each"                                                                                                                                                 | dnn-extra-layer               | 2000                         | 200              |
| 6      | "3 layers, 64 neuron each"                                                                                                                                                 | dnn-extra-layer               | 3500                         | 100              |
| 7      | "2D conv/pool layer (32 filters, kernel size 3, 1 layer), 2D conv/pool layer (16 filters, kernel size 3, 1 layer), "                                                       | cnn-base                      | 50                           | 200              |
| 8      | "2D conv/pool layer (32 filters, kernel size 5, 1 layer), 2D conv/pool layer (16 filters, kernel size 5, 1 layer), "                                                       | cnn-larger-kernel             | 50                           | 200              |
| 9      | "2D conv/pool layer (32 filters, kernel size 5, 1 layer), 2D conv/pool layer (16 filters, kernel size 5, 1 layer), 2D conv/pool layer (8 filters, kernel size 5, 1 layer)" | cnn-larger-kernel-extra-layer | 50                           | 200              |
| 10     | "2D conv/pool layer (64 filters, kernel size 3, 1 layer), 2D conv/pool layer (32 filters, kernel size 3, 1 layer), "                                                       | cnn-double-filters            | 50                           | 200              |
| 11     | "2D conv/pool layer (32 filters, kernel size 3, 1 layer), 2D conv/pool layer (16 filters, kernel size 3, 1 layer), "                                                       | cnn-base                      | 2000                         | 50               |
| 12     | "2D conv/pool layer (32 filters, kernel size 3, 1 layer), 2D conv/pool layer (16 filters, kernel size 3, 1 layer), "                                                       | cnn-base                      | 4000                         | 50               |


![](assets/images/memory-usage.png?raw=true "memory-usage")
![](assets/images/accuracy.png?raw=true "accuracy")

Take-aways:

* Inference times with CNN models are (at least) 10x larger than DNN models
* CNN models require more RAM
* Increasing the kernel size in CNN models has a relatively small impact in memory usage, but significantly increases inference time
* Increasing the number of filters in CNN models has a significant impact both in inference time and memory usage, without a significant gain in accuracy.
* A DNN can attain similar accuracy values to that of a CNN by duplicating the number of neurons at each layer (64 to 128 neurons). However, this implies high memory usage costs.

### Live inference performance

#### Accuracy & sensitivity analysis

After deploying the first DNN model in the Arduino and running live inference, I've noticed that the classification scores where highly sensitive to the position of the object in the frame.

As such, I've conducted a simple sensitivity analysis using the best performing DNN and CNN models, using an augmented dataset that translates a non-rotated centered object around the frame, along (x,y) coordinates.

The position sensitivity test consists in deploying models 4 (DNN) and 12 (CNN) in the Arduino, and recording the classification scores on 9 positions in the frame, using a total of 12 different electronic components (4 per non-background class), as shown in the image below.

![](assets/images/test-samples.png?raw=true "test-samples")

The graph below shows the distributions of the scores assigned to the 'true label', for both the DNN and CNN models.

#### Saliency Map comparision between DNN and CNN classifiers

#### GRAD-CAM of CNN classifiers

