#!/usr/bin/env python
import h5py
filename = 'weights-best.hdf5'

print("File handling with h5py")
f = h5py.File(filename, 'r')

a_group_key = list(f.keys())[0]

print(f.keys())
data = list(f[a_group_key])

print(data)

print("File handling with keras")
from keras.models import load_model
model = load_model(filename)

#Define architecture
def create_model():
   model = Sequential()
   model.add(Dense(64, input_dim=14, init='uniform'))
   model.add(LeakyReLU(alpha=0.3))
   model.add(BatchNormalization(epsilon=1e-06, mode=0, momentum=0.9, weights=None))
   model.add(Dropout(0.5)) 
   model.add(Dense(64, init='uniform'))
   model.add(LeakyReLU(alpha=0.3))
   model.add(BatchNormalization(epsilon=1e-06, mode=0, momentum=0.9, weights=None))
   model.add(Dropout(0.5))
   model.add(Dense(2, init='uniform'))
   model.add(Activation('softmax'))
   return model

def train():
   model = create_model()
   sgd = SGD(lr=0.1, decay=1e-6, momentum=0.9, nesterov=True)
   model.compile(loss='binary_crossentropy', optimizer=sgd)

   checkpointer = ModelCheckpoint(filepath=filename, verbose=1, save_best_only=True)
   model.fit(X_train, y_train, nb_epoch=20, batch_size=16, show_accuracy=True, validation_split=0.2, verbose=2, callbacks=[checkpointer])

def load_trained_model(weights_path):
   model = create_model()
   model.load_weights(weights_path)

model = load_trained_model(filename)
