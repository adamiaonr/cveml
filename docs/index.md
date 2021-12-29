This page showcases my projects while taking the Coursera [Computer Vision with Embedded Machine Learning](https://www.coursera.org/learn/computer-vision-with-embedded-machine-learning) course, taught by Shawn Hymmel and supported by the [Edge Impulse](https://www.edgeimpulse.com/) online tool.

### Why should you care?

Is this another standard 'hand-in' from a course project? Not quite:

* Instead of using the recommended hardware platforms for the course as of November 2021 (i.e., OpenMV Camera or Raspberry Pi 4 with PiCamera), I've used [an Arduino Nano BLE 33 Sense](https://docs.arduino.cc/hardware/nano-33-ble-sense) connected to an [OV7670 CMOS camera](https://www.openhacks.com/uploadsproductos/ov7670_cmos_camera_module_revc_ds.pdf). 
* This work shows how to create a data capture and live inference setup with the Arduino + OV7670 camera, and also how to integrate the TensorFlow Lite models trained in Edge Impulse in Arduino.
* I've followed the proposed 'electronic component' classification task, but decided to make it harder by using 4 to 5 different components per class. E.g., I've built the LED dataset with photos from 4 different LEDs, the capacitor dataset with 5 different capacitors, etc.
* Instead of simply following the tutorial script, I've experimented with different model types and hyperparameter sets, comparing the live performance in the chosen classification task along two metrics: (i) accuracy and (ii) sensitivity to the position of the object in the frame. The results can be interesting to you, and are shown below.

## Analysis

### Models, hyperparameters and datasets

I've recorded the performance of multiple Neural Network models, changing hyperparameters and using different dataset versions. A summary of the models I've experimented with is shown [here](../projects/project-001/README.md).

#### Summary

* DNNs provide higher accuracies for small datasets, but CNNs are superior to DNNs as we augment the dataset
* For the larger datasets, the accuracy of a DNN can approximate that of a CNN by increasing the number of neurons at each layer (e.g., 64 to 128 neurons). However, DNNs end up costing more in terms of memory (considering the aggregate of peak RAM usage and flash memory), while still underperforming CNNs for the same dataset.
* Inference times with CNN models are (at least) 10x larger than DNN models
* CNN models require more RAM than DNN models
* Larger kernel sizes in CNN models have a relatively small impact in memory usage, but significantly increases inference time
* Adding more filters to CNN models has a significant impact in both in inference time and memory usage, with low accuracy gains

#### Inference times and memory usage

The estimated inference time (using a Cortex M4F 80 MHz CPU as reference), RAM and flash usage is shown in the images below:

![](assets/images/memory-usage.png?raw=true "memory-usage")

#### Model accuracy

##### Dataset: `base`

![](assets/images/accuracy-base.png?raw=true "accuracy")

##### Dataset: `augmented-transl-rot`

![](assets/images/accuracy-augmented-transl-rot.png?raw=true "accuracy")

##### Dataset: `augmented-transl-larger`

![](assets/images/accuracy-augmented-transl-larger.png?raw=true "accuracy")

### Live inference performance

For the live inference performance, I've compared a DNN to a CNN model, both trained with the `augmented-transl-larger` dataset:

* DNN : dnn-2x-neuron config (model 4)
* CNN : cnn-base config (model 12)

I've tested both models by classifying 12 different electronic components (4 per non-background class), as shown in the image below. Furthermore, I've recorded the class prediction scores for each component when positioned at 9 different positions in the frame. The goal was to understand how sensitive each classifier is to movements of the objects in the frame.  

![](assets/images/test-samples.png?raw=true "test-samples")

#### Accuracy

No surprises here : the CNN model achieved the highest live inference performance:

* **DNN :** 75.83%
* **CNN :** 96.67%

#### Sensitivity analysis

The table below shows the F1 scores per class, for the DNN and CNN models. Lower F1 scores mean that a certain {model, class} tuple is more sensitive to changes of position in the frame.

##### Summary:

* The CNN model is less sensitive to variations in position of the object in the frame 
* The analysis of saliency maps generated from a selected set of samples (see below) shows that the CNN classifier looks at very specific pixels for each class, always in the position relative to the object. 
* In contrast, the saliency maps of the DNN classifier appear more 'chaotic' (I've adapted the 'CNN visualization' notebook to create saliency maps for DNNs, [here]())

##### DNN classifier:

            | background | capacitor          | led                | resistor           
------------|------------|--------------------|--------------------|--------------------
 **background** | 12.0       | 0.0                | 0.0                | 0.0                
 **capacitor**  | 0.0        | 25.0               | 10.0               | 1.0                
 **led**        | 0.0        | 7.0                | 29.0               | 0.0                
 **resistor**   | 0.0        | 0.0                | 6.0                | 30.0               
 **f1 score**   | **1.000**        | **0.735** | **0.716** | **0.895**
 
##### CNN classifier:

            | background | capacitor | led   | resistor 
------------|------------|-----------|-------|----------
 **background** | 12.0       | 0.0       | 0.0   | 0.0      
 **capacitor  | 0.0        | 35.0      | 0.0   | 1.0      
 **led**        | 1.0        | 2.0       | 33.0  | 0.0      
 **resistor**   | 0.0        | 0.0       | 0.0   | 36.0     
 **f1 score**   | **0.960**      | **0.959**     | **0.957** | **0.986**    


##### Saliency map for CNN classifier

The title of each image should be read as `<sample-id> : <true-label> (<predicted-label>)`.

![](assets/images/saliency-maps-cnn.png?raw=true "saliency-cnn")

##### Saliency map for DNN classifier

The title of each image should be read as `<sample-id> : <true-label> (<predicted-label>)`.

![](assets/images/saliency-maps-dnn.png?raw=true "saliency-dnn")

#### GRAD-CAM of CNN classifiers

The title of each image should be read as `<sample-id> : <true-label> (<predicted-label>)`.

![](assets/images/grad-cam-heatmaps-overlay-cnn.png?raw=true "saliency-dnn")
