import pickle
from common_functions import *

############################################################################################################

def process_video():

    input_clip_file = "project_video.mp4"
    output_clip_file = "project_video_with_tracking.mp4"

    #input_clip = VideoFileClip(input_clip_file)
    #output_clip = input_clip.fl_image(process_video_frame)
    #output_clip.write_videofile(output_clip_file, audio=False)

############################################################################################################

trained_model = pickle.load(open("model.p", "rb"))
clf = trained_model["clf"]
scaler = trained_model["scaler"]
process_images(clf, scaler)