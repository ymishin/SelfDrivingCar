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

The initial model architecture was LeNet-5. Different values of hyperparameters were evaluated (batch size, number of epochs and learning rate) but it was not possible to attain desired validation set accuracy. Thus the model was substantially increased. The final model contains 2 convolution and 3 fully connected layers as LeNet-5, but all layers are larger. Besides, two dropout layers were added to help with possible overfitting problems.

### 3. Model Testing on New Images.

#### 1. Choose five German traffic signs found on the web and provide them in the report. For each image, discuss what quality or qualities might be difficult to classify.

Here are five German traffic signs that I found on the web:

![alt text][image4] ![alt text][image5] ![alt text][image6] 
![alt text][image7] ![alt text][image8]

The first image might be difficult to classify because ...

#### 2. Discuss the model's predictions on these new traffic signs and compare the results to predicting on the test set. At a minimum, discuss what the predictions were, the accuracy on these new predictions, and compare the accuracy to the accuracy on the test set.

Here are the results of the prediction:

| Image			        |     Prediction	        					| 
|:---------------------:|:---------------------------------------------:| 
| Stop Sign      		| Stop sign   									| 
| U-turn     			| U-turn 										|
| Yield					| Yield											|
| 100 km/h	      		| Bumpy Road					 				|
| Slippery Road			| Slippery Road      							|


The model was able to correctly guess 4 of the 5 traffic signs, which gives an accuracy of 80%. This compares favorably to the accuracy on the test set of ...

#### 3. Describe how certain the model is when predicting on each of the five new images by looking at the softmax probabilities for each prediction. Provide the top 5 softmax probabilities for each image along with the sign type of each probability. (OPTIONAL: as described in the "Stand Out Suggestions" part of the rubric, visualizations can also be provided such as bar charts)

The code for making predictions on my final model is located in the 11th cell of the Ipython notebook.

For the first image, the model is relatively sure that this is a stop sign (probability of 0.6), and the image does contain a stop sign. The top five soft max probabilities were

| Probability         	|     Prediction	        					| 
|:---------------------:|:---------------------------------------------:| 
| .60         			| Stop sign   									| 
| .20     				| U-turn 										|
| .05					| Yield											|
| .04	      			| Bumpy Road					 				|
| .01				    | Slippery Road      							|


For the second image ... 



