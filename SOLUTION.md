# Lösung der Aufgabe 1 - Excercise ROS + Supervised Learning

Exercise ROS + Supervised Learning ist Pflichtbeleg im Kurs "K.I. in der Robotik" des Bachelorstudiengangs Angewandte Informatik an der Hochschule für Technik und Wirtschaft (HTW). 
Ziel des Belegs ist das Üben der Nachrichtenverwaltung zwischen Prozessen (Nodes) anhand des Publisher-Subscriber Patterns. Als Nachrichten werden Vorhersagen bezüglich eines Datensatzes (MNIST) geschickt sowie nach Richtigkeit überprüft.

## Supervised Learning

Daten stammen aus dem MNIST-Datensatz. Darüber wird ein CNN (Convolutional Neural Network) auf der Basis der KERAS API mit Tensorflow Backend trainiert. Das Modell liefert eine Genauigkeit von xx% nach yy Epochen.

Das Modell ist wie folgt aufgebaut:

~~~~
model = Sequential()
model.add(Conv2D(filters=32,
                 kernel_size=(3, 3),
                 activation='relu',
                 input_shape=input_shape))
model.add(Conv2D(filters=64,
                 kernel_size=(3, 3),
                 activation='relu'))
model.add(MaxPooling2D(pool_size=(2, 2)))
model.add(Dropout(rate=0.25))
model.add(Flatten())
model.add(Dense(units=128,
                activation='relu'))
model.add(Dropout(rate=0.5))
model.add(Dense(units=num_classes,
                activation='softmax'))
~~~~
Es besteht aus 2 Convulational Layern (2D), beide mit Kernel-Size (3,3) und ReLu-Aktivierungsfunktion, sowie aus einem MaxPooling Layer (2D), Dropout, Flatten und Dense Layern.

Wichtig hierbei ist, dass die Input-Dimension von der Keras-API akzeptabel ist und mit der Dimension der Input-Daten übereinstimmt. Diese ist bei den MNIST-Datensatz: 28x28. Da es um schwarz-weiße Bilder geht, ist die Channel-Dimension 1.

Nach dem Training wird das Modell als `mnist_cnn.hdf5`

## 1. Aufgabenstellung

## 2. ROS - Publisher/Subscriber Pattern

## 3. Das KERAS-Modell



## 4. Implementierung: Kommunikation zwischen den ROS-Nodes
