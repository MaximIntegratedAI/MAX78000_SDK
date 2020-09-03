# MAX78000 FaceID Demo



## Overview
The FaceID Demo software demonstrates identification of a number of persons from their facial images using MAX78000 EVKIT.

For this purpose, the CNN model generates a 512-length embedding for a given image, whose distance to whole embeddings stored for each subject is calculated. The image is identified as either one of these subjects or 'Unknown' depending on the embedding distances.

The CNN model is trained with the VGGFace-2 dataset using MTCNN and FaceNet models for embedding generation.

## FaceID Demo Software

### Building firmware:

Navigate directory where FaceID demo software is located and build the project:

```bash
$ cd /Examples/MAX78000/CNN/faceid_demo
$ make
```

If this is the first time after installing tools, or peripheral files have been updated, first clean drivers before rebuilding the project: 

```bash
$ make distclean
```

### Load firmware image to target:

Connect USB cable to CN1 (USB/PWR) and turn ON power switch (SW1). Note the COM port (COM_PORT) of this connection from your system configuration.

Connect PICO adapter to JH5 SWD header. 

Load firmware image using Openocd. **Make sure to remove PICO adapter once firmware is loaded.**

```bash
./openocd -f tcl/interface/cmsis-dap.cfg -f tcl/target/max78000.cfg -c "program build/MAX78000.elf verify reset exit"
```

### Face Database Generation:

#### Prerequisites:
- Python 3.6.9 or higher (tested for 3.7.7 and 3.8.1)
- NumPy (>=1.19)
- Scipy (>=1.4)
- OpenCv (>=3.4)
- PySerial (>=3.4)
- MatplotLib (>=3.2)
- PyTorch (>=1.4.0)
- Torchvision (>=0.5.0)

#### Populate Face Images
Navigate to 'db' folder and create a folder for each subject in your database and copy each subject's photos into these folders. These photos must contain subject's face directed to the camera but do not need to be consisting only the face. The structure of the directory should be as the following.

```bash
├── db
    ├── Name1
    │    ├── Image1.jpg
    │    ├── Image2.jpg
    │    ├── Image3.jpg
    │    ├── Image4.jpg
    │    ├── Image5.jpg
    │    └── Image6.jpg
    └── Name2
         ├── Image1.jpg
         ├── Image2.jpg
         ├── Image3.jpg
         ├── Image4.jpg
         └── Image5.jpg
```

Having at least 5 images per subject is recommended. There must be no other person in the images besides the subject. For the both the images in the database and in the operation time, the face should be well and evenly illuminated.

#### Generate DB

Run the python script by:

```bash
$ python db_gen/generate_face_db.py --db db --db-filename embeddings --include-path include
```

The script updates embeddings.h file under 'include' folder using the images under 'db' folder. 

## CNN Model Design
### Problem Definition
To identify people from 3 channel (RGB) frontal facial images, i.e. portraits. Small amount of rotations should be considered to have a robust application.

### Approach
The main approach in the literature is composed of three steps:

- Face Extraction: Detection of the faces in the image and extract a rectangular subimage that contains only face.
- Face Alignment: The rotation angles (in 3D) of the face in the image is find to compensate its effect by affine transformation.
- Face Identification: The extracted subimage is used to identify the person.

In this project, the aim is to run all those steps in a single AI-85 chip so the approach is to identify the faces with a single from uncropped portraits, each contains 1 face only.

Then, the embeddings (Face ID) are created by FaceNet [2] model as seen below and used these embeddings as our target. There is no need to deal with center loss, triplet loss etc, since those are assumed to be covered by FaceNet model. The loss used in the model development will be Mean Square Error (MSE) between the target and predicted embeddings.

### CNN Model
The CNN model synthesized for MAX78000 is 9 layer sequential [model](db_gen/ai85/AI85FaceIDNetNoBias.py). It takes 160x120 RGB image from the input and gives out 512 length embedding corresponds to the image.


## References
[1] MTCNN: https://arxiv.org/ftp/arxiv/papers/1604/1604.02878.pdf

[2] FaceNet: https://arxiv.org/pdf/1503.03832.pdf
