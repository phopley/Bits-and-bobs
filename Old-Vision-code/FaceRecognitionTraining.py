# Produces face recognition data from .jpg images (training data) and stores it on
# disk for retrevial by actual face regcognition code.
# code based on informramiz http://github.com/informramiz/opencv-face-reccognition-python
#
# The more images used in training the better.
# Normally a lot of images are used for training a face recognizer so that it can learn different
# looks of the same person, for example with glasses, without glasses, laughing, sad, happy,
# crying, with beard, without beard etc.
# 
# All training data should be inside a directory called 'training-data'. This directory contains
# one folder for each person, the name of this directory is in the format 'sLabel' (e.g. s1, s2)
# where label is actually the integer label assigned to that person.
# The directory structure tree for training data is as follows:
# 
# training-data
# |-------------- s1
# |               |-- 1.jpg
# |               |-- ...
# |               |-- 12.jpg
# |-------------- s2
# |               |-- 1.jpg
# |               |-- ...
# |               |-- 12.jpg
# 

# Import OpenCV module
import cv2
# Import os module for file access
import os
#import numpy to convert python lists to numpy arrays for OpenCV and for saving the data 
import numpy as np
#import pickle to save list of names
import pickle

# #####################
# Prepare training data
# #####################

# This function detects a face using OpenCV from the supplied image
def detect_face(img):
    #convert the test image to gray image as opencv face detector expects gray images
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    #load OpenCV face detector, I am using LBP which is fast
    #there is also a more accurate but slow Haar classifier
    face_cascade = cv2.CascadeClassifier('opencv-files/haarcascade_frontalface_alt.xml')

    #let's detect multiscale (some images may be closer to camera than others) images
    #result is a list of faces
    faces = face_cascade.detectMultiScale(gray, 1.1, 5);
    
    #if no faces are detected then return None
    if (len(faces) == 0):
        return None, None
    
    #under the assumption that there will be only one face in each training image
    #extract the face area from 
    (x, y, w, h) = faces[0]
    
    #return only the face part of the image
    return gray[y:y+w, x:x+h], faces[0]


# This function reads all the persons training images, attempts to detect a face from each image
# and will return two lists of exactly same size, one list of faces and another list of labels for
# each face
def prepare_training_data(data_folder_path):
    
    #------STEP-1--------
    #get the directories (one directory for each subject) in data folder
    dirs = os.listdir(data_folder_path)
    
    #list to hold all subject faces
    faces = []
    #list to hold labels for all subjects
    labels = []
    #list to hold names
    names = ['?']
               
    # Go through each directory and create a space in names
    for dir_name in dirs:
        if dir_name.startswith("s"):
            label = int(dir_name.replace("s", ""))
            names.append('?')
        
    
    #let's go through each directory and read images within it
    for dir_name in dirs:
        
        #our subject directories start with letter 's' so
        #ignore any non-relevant directories if any
        if not dir_name.startswith("s"):
            continue;
            
        #------STEP-2--------
        #extract label number of subject from dir_name
        #format of dir name = slabel
        #, so removing letter 's' from dir_name will give us label
        label = int(dir_name.replace("s", ""))
                
        #build path of directory containin images for current subject subject
        #sample subject_dir_path = "training-data/s1"
        subject_dir_path = data_folder_path + "/" + dir_name
        
        #get the images names that are inside the given subject directory
        subject_images_names = os.listdir(subject_dir_path)
        
        #------STEP-3--------
        #go through each image name, read image, 
        #detect face and add face to list of faces
        for image_name in subject_images_names:
            
            #ignore system files like .DS_Store
            if image_name.startswith("."):
                continue;
            
            #build image path
            #sample image path = training-data/s1/1.pgm
            image_path = subject_dir_path + "/" + image_name

            #read image
            image = cv2.imread(image_path)
            
            #display an image window to show the image
            # Future code will only display the first image of each subject and prompt for name entry
            cv2.imshow("Training on image...", cv2.resize(image, (400, 500)))            
            cv2.waitKey(100)

            # Check if we have a name for this subject
            if names[label] == '?':                
                names[label] = raw_input("What is this persons name? ");

            
            #detect face
            face, rect = detect_face(image)
            
            #------STEP-4--------
            # We will ignore faces that are not detected
            if face is not None:
                #add face to list of faces
                faces.append(face)
                #add label for this face
                labels.append(label)
            
    cv2.destroyAllWindows()
    cv2.waitKey(1)
    cv2.destroyAllWindows()
    
    return faces, labels, names


print("Preparing data...")
faces, labels, names = prepare_training_data("training-data")
print("Data prepared")

#print total faces and labels
print("Total faces: ", len(faces))
print("Total labels: ", len(labels))

# Save the data created as numpy files
np.save('faces-data/faces', faces, allow_pickle=True, fix_imports=False)
np.save('faces-data/labels', np.array(labels), allow_pickle=True, fix_imports=False)
# Save the names
with open ('faces-data/names', 'wb') as fp:
    pickle.dump(names, fp)

