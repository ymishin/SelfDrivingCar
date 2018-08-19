# **Traffic Sign Recognition** 

---

**The goal of this project is to design and apply traffic sign classifier using TensorFlow**

---

### 1. Data Set Summary & Exploration.

#### Basic Summary of the Data Set.

Summary statistics of the traffic signs data set:

* Number of training samples = 34799
* Number of validation samples = 4410
* Number of testing samples = 12630
* Image data shape = (32, 32, 3)
* Number of classes = 43

#### Eploratory visualization of the dataset.

Here are distributions of labels in training, validation and testing sets:

<img src="./images/distribution_train.jpg">
<img src="./images/distribution_valid.jpg">
<img src="./images/distribution_test.jpg">

Some random image samples from training set:

<img src="./images/random_signs.jpg">

### 2. Model Architecture Design and Testing.

#### Image Data Preprocessing.

Image data were normalized to zero mean and unit variance.

#### CNN architecture.

The network consists of the following layers:

| Layer         		      |     Description	        					                 | 
|:----------------------|:----------------------------------------------| 
| Input               		| 32x32x3 normalized RGB image   						        	| 
| Convolution 5x5      	| 1x1 stride, valid padding, outputs 28x28x12  	|
| RELU	             				|	Activation layer                             	|
| Max pooling	         	| 2x2 stride,  outputs 14x14x12             				|
| Convolution 5x5      	| 1x1 stride, valid padding, outputs 10x10x24  	|
| RELU	             				|	Activation layer                             	|
| Max pooling	         	| 2x2 stride,  outputs 5x5x24               				|
| Fully connected	     	| Outputs 180                          									|
| RELU	             				|	Activation layer                             	|
| Dropout           				|	Regularization layer                         	|
| Fully connected	     	| Outputs 130                          									|
| RELU	             				|	Activation layer                             	|
| Dropout           				|	Regularization layer                         	|
| Fully connected	     	| Outputs 43                           									| 

#### CNN training and evaluation.

The CNN was trained using the following parameters:

* Number of epochs = 10
* Batch size = 128
* Keep probabilty for dropout layers = 0.5
* AdamOptimizer, learning rate = 0.001

Results of training and evaluation are:

* Validation set accuracy = 0.959
* Test set accuracy = 0.944

#### Some notes about the choosen model architecture and hyperparameters.

The initial model architecture was equivalent to LeNet-5, except output layer was changed to predict 43 classes. Different values of hyperparameters were evaluated (batch size, number of epochs and learning rate) but it was not possible to attain desired validation set accuracy. Thus the model was substantially increased. The final model contains 2 convolution and 3 fully connected layers as LeNet-5, but all layers are larger. Besides, two dropout layers were added to help with possible overfitting problems.

### 3. Model Testing on New Images.

#### 5 random traffic signs from the web.

<p float="left">
<img src="./data/slippery_road.jpg" width="64">
<img src="./data/80_kmh.jpg" width="64">
<img src="./data/120_kmh.jpg" width="64">
<img src="./data/stop.jpg" width="64">
<img src="./data/road_work.jpg" width="64">
</p>

All these images should be relatively easy to classify. "80 km/h" sign could be a little problematic as it is rotated a little bit.

#### CNN classification of new traffic signs images.

All images were correctly classified with surpsingly high accuracy. Here are classification probablities:

| Image			        | Classification probability  | 
|:------------------|:----------------------------| 
| Slippery road     | 0.99   		    							| 
| 80 km/h      			| 0.98       								  |
| 120 km/h  				| 1.00                        |
| Stop          		| 0.99                        |
| Road work    			| 0.96          							|

This gives model accuracy of 100% on this data set.
