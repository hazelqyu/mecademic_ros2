import cv2
import numpy as np
from keras.models import load_model
import mediapipe as mp

# Initialize MediaPipe for face detection
mp_face_detection = mp.solutions.face_detection.FaceDetection(min_detection_confidence=0.5)

# Load the pre-trained FER2013 model (mini-XCEPTION trained on FER2013)
model = load_model('fer2013_big_XCEPTION.hdf5')

# Define the emotion labels for FER2013 (7 classes)
emotion_labels = ['Angry', 'Disgust', 'Fear', 'Happy', 'Sad', 'Surprise', 'Neutral']

# Function to preprocess face image for FER2013 model
def preprocess_face(face_image):
    face_image = cv2.cvtColor(face_image, cv2.COLOR_BGR2GRAY)  # Convert face image to grayscale
    face_image = cv2.resize(face_image, (64, 64))  # Resize to 64x64 (FER2013 input size)
    face_image = face_image.astype('float32') / 255  # Normalize pixel values to [0, 1]
    face_image = np.expand_dims(face_image, axis=0)  # Add batch dimension
    face_image = np.expand_dims(face_image, axis=-1)  # Add channel dimension (grayscale)
    return face_image

# Function to recognize emotion using the FER2013 model
def recognize_expression(face_image):
    preprocessed_face = preprocess_face(face_image)
    predictions = model.predict(preprocessed_face)  # Get model predictions
    emotion_index = np.argmax(predictions)  # Find the highest confidence index
    return emotion_labels[emotion_index], np.max(predictions)  # Return emotion label and confidence

# Open the webcam or video feed
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert the frame to RGB for MediaPipe
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Detect faces using MediaPipe
    results = mp_face_detection.process(rgb_frame)

    if results.detections:
        for detection in results.detections:
            bboxC = detection.location_data.relative_bounding_box
            ih, iw, _ = frame.shape
            x = int(bboxC.xmin * iw)
            y = int(bboxC.ymin * ih)
            w = int(bboxC.width * iw)
            h = int(bboxC.height * ih)

            # Extract the face from the frame
            face_image = frame[y:y+h, x:x+w]

            # Recognize the emotion using the FER2013 model
            emotion, confidence = recognize_expression(face_image)

            # Only display the emotion and confidence if they are not None
            if emotion and confidence is not None:
                # Draw the bounding box and emotion on the frame
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                cv2.putText(frame, f'{emotion} ({confidence:.2f})', (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36, 255, 12), 2)
            else:
                # Optionally, show a default message if no emotion is detected
                cv2.putText(frame, 'No emotion detected', (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

    # Display the frame with face detection and emotion recognition
    cv2.imshow('Facial Tracking and Emotion Recognition', frame)

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture and close windows
cap.release()
cv2.destroyAllWindows()
