from tensorflow.keras.models import load_model
from tensorflow.keras.optimizers import Adam

# Load the model without compiling
model = load_model('/home/andrek/ros2_ws/fer2013_big_XCEPTION.hdf5', compile=False)
# Set a compatible optimizer
model.compile(optimizer=Adam(learning_rate=0.0001), loss='categorical_crossentropy')
model.save('/home/andrek/ros2_ws/emotion _detection.keras')
