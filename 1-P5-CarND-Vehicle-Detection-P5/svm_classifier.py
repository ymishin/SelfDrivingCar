import pickle
from common_functions import *

############################################################################################################

#unzip_data()
car_images, noncar_images = read_data()
car_features = extract_features(car_images)
noncar_features = extract_features(noncar_images)
clf, scaler = train_svm_classifier(car_features, noncar_features)
trained_model = {"clf": clf, "scaler": scaler}
pickle.dump(trained_model, open("model.p", "wb"))