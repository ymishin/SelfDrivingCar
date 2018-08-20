import pickle
from common_functions import *

############################################################################################################

trained_model = pickle.load(open("model.p", "rb"))
clf = trained_model["clf"]
scaler = trained_model["scaler"]
process_images(clf, scaler)
process_video(clf, scaler)