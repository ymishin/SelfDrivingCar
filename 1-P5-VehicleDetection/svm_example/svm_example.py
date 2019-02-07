import sys
import matplotlib.pyplot as plt
import copy
import numpy as np
import pylab as pl
from sklearn.svm import SVC
from sklearn.metrics import accuracy_score
from class_vis import prettyPicture
from prep_terrain_data import makeTerrainData

features_train, labels_train, features_test, labels_test = makeTerrainData()

########################## SVM #################################

# SVM classifier
clf = SVC(kernel="rbf", gamma=5.0)

# Fit the classifier
clf.fit(features_train, labels_train)

# Accuracy
pred = clf.predict(features_test)
acc = accuracy_score(pred, labels_test)
print("Accuracy: ", acc)

# Visualize on test data
prettyPicture(clf, features_test, labels_test)