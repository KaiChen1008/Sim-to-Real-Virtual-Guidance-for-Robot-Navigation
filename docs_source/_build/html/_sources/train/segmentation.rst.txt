How to Train a Segmentation Model
=================================


Installation
------------

**Install Google Deeplab**


We use `Google DeepLab <https://github.com/tensorflow/models/tree/master/research/deeplab>`_ to train our model. 
Follow the installation instructions `here <https://github.com/tensorflow/models/blob/master/research/deeplab/g3doc/installation.md>`_ . 
``tensorflow-gpu==1.12.0`` and ``cuda version = 9.0`` is recommended if an effor occurs while installing.



**Download the dataset**


We use `Cityscapes Dataset <https://www.cityscapes-dataset.com/>`_ for outdoor training data, 
and `ADE20K Dataset <https://groups.csail.mit.edu/vision/datasets/ADE20K/>`_  for indoor training data. 
To download Cityscapes, register the `website <https://www.cityscapes-dataset.com/>`_ and download manualy or use

.. code-block:: bash

    wget --keep-session-cookies --save-cookies=cookies.txt --post-data 'username=yourusername&password=yourpassword&submit=Login' https://www.cityscapes-dataset.com/login/
    wget --load-cookies cookies.txt --content-disposition https://www.cityscapes-dataset.com/file-handling/?packageID=1,3


where the package ID needed are ``1`` and ``3``.


Follow the steps bellow to download ADE20K dataset.


Run the dataset


Steps to run Cityscapes can be found `here <https://github.com/tensorflow/models/blob/master/research/deeplab/g3doc/cityscapes.md>`_ .


Steps to run and download ADE20K can be found `here <https://github.com/tensorflow/models/blob/master/research/deeplab/g3doc/ade20k.md>`_ .


We use initial checkpoint m`obilenetv2_coco_cityscapes_trainfine <http://download.tensorflow.org/models/deeplabv3_mnv2_cityscapes_train_2018_02_05.tar.gz>`_  
provided by `TensorFlow DeepLab Model Zoo <https://github.com/tensorflow/models/blob/master/research/deeplab/g3doc/model_zoo.md>`_ .


Our parameters are


.. code-block:: bash

    python deeplab/train.py \
    --logtostderr \
    --training_number_of_steps=90000 \
    --train_split="train" \
    --model_variant="mobilenet_v2" \
    --output_stride=16 \
    --train_crop_size=321 \
    --train_crop_size=321 \
    --train_batch_size=16 \
    --fine_tune_batch_norm=true \
    --dataset="cityscapes" \
    --tf_initial_checkpoint=${PATH_TO_INITIAL_CHECKPOINT} \
    --train_logdir=${PATH_TO_TRAIN_DIR} \
    --dataset_dir=${PATH_TO_DATASET}



**Train your own data**


In addition to Cityscapes dataset, we also trained on images collected and annotated by students on our own campas. Add images to Cityscapes dataset in gtfine and leftImg8bit folders, and convert to TFRecord with convert_cityscapes.sh. Remember to convert annotation colors to trainIDs.
A dictionary of trainIDs and anotation colors is shown below.


.. code-block:: bash

    Label = namedtuple( 'Label' , ['name','id','trainId','category','categoryId','hasInstances','ignoreInEval','color',] )
    labels = [
        #       name                     id    trainId   category            catId     hasInstances   ignoreInEval   color
        Label(  'unlabeled'            ,  0 ,      255 , 'void'            , 0       , False        , True         , (  0,  0,  0) ),
        Label(  'ego vehicle'          ,  1 ,      255 , 'void'            , 0       , False        , True         , (  0,  0,  0) ),
        Label(  'rectification border' ,  2 ,      255 , 'void'            , 0       , False        , True         , (  0,  0,  0) ),
        Label(  'out of roi'           ,  3 ,      255 , 'void'            , 0       , False        , True         , (  0,  0,  0) ),
        Label(  'static'               ,  4 ,      255 , 'void'            , 0       , False        , True         , (  0,  0,  0) ),
        Label(  'dynamic'              ,  5 ,      255 , 'void'            , 0       , False        , True         , (111, 74,  0) ),
        Label(  'ground'               ,  6 ,      255 , 'void'            , 0       , False        , True         , ( 81,  0, 81) ),
        Label(  'road'                 ,  7 ,        0 , 'flat'            , 1       , False        , False        , (128, 64,128) ),
        Label(  'sidewalk'             ,  8 ,        1 , 'flat'            , 1       , False        , False        , (244, 35,232) ),
        Label(  'parking'              ,  9 ,      255 , 'flat'            , 1       , False        , True         , (250,170,160) ),
        Label(  'rail track'           , 10 ,      255 , 'flat'            , 1       , False        , True         , (230,150,140) ),
        Label(  'building'             , 11 ,        2 , 'construction'    , 2       , False        , False        , ( 70, 70, 70) ),
        Label(  'wall'                 , 12 ,        3 , 'construction'    , 2       , False        , False        , (102,102,156) ),
        Label(  'fence'                , 13 ,        4 , 'construction'    , 2       , False        , False        , (190,153,153) ),
        Label(  'guard rail'           , 14 ,      255 , 'construction'    , 2       , False        , True         , (180,165,180) ),
        Label(  'bridge'               , 15 ,      255 , 'construction'    , 2       , False        , True         , (150,100,100) ),
        Label(  'tunnel'               , 16 ,      255 , 'construction'    , 2       , False        , True         , (150,120, 90) ),
        Label(  'pole'                 , 17 ,        5 , 'object'          , 3       , False        , False        , (153,153,153) ),
        Label(  'polegroup'            , 18 ,      255 , 'object'          , 3       , False        , True         , (153,153,153) ),
        Label(  'traffic light'        , 19 ,        6 , 'object'          , 3       , False        , False        , (250,170, 30) ),
        Label(  'traffic sign'         , 20 ,        7 , 'object'          , 3       , False        , False        , (220,220,  0) ),
        Label(  'vegetation'           , 21 ,        8 , 'nature'          , 4       , False        , False        , (107,142, 35) ),
        Label(  'terrain'              , 22 ,        9 , 'nature'          , 4       , False        , False        , (152,251,152) ),
        Label(  'sky'                  , 23 ,       10 , 'sky'             , 5       , False        , False        , ( 70,130,180) ),
        Label(  'person'               , 24 ,       11 , 'human'           , 6       , True         , False        , (220, 20, 60) ),
        Label(  'rider'                , 25 ,       12 , 'human'           , 6       , True         , False        , (255,  0,  0) ),
        Label(  'car'                  , 26 ,       13 , 'vehicle'         , 7       , True         , False        , (  0,  0,142) ),
        Label(  'truck'                , 27 ,       14 , 'vehicle'         , 7       , True         , False        , (  0,  0, 70) ),
        Label(  'bus'                  , 28 ,       15 , 'vehicle'         , 7       , True         , False        , (  0, 60,100) ),
        Label(  'caravan'              , 29 ,      255 , 'vehicle'         , 7       , True         , True         , (  0,  0, 90) ),
        Label(  'trailer'              , 30 ,      255 , 'vehicle'         , 7       , True         , True         , (  0,  0,110) ),
        Label(  'train'                , 31 ,       16 , 'vehicle'         , 7       , True         , False        , (  0, 80,100) ),
        Label(  'motorcycle'           , 32 ,       17 , 'vehicle'         , 7       , True         , False        , (  0,  0,230) ),
        Label(  'bicycle'              , 33 ,       18 , 'vehicle'         , 7       , True         , False        , (119, 11, 32) ),
        Label(  'license plate'        , -1 ,       -1 , 'vehicle'         , 7       , False        , True         , (  0,  0,142) ),
    ]


**Using your trained model**

After exporting your model, tar frozen_inference_graph.pb with


.. code-block:: bash

    tar -zcvf mymodel.tar.gz frozen_inference_graph.pb



An inference example can be found `here <https://colab.research.google.com/github/tensorflow/models/blob/master/research/deeplab/deeplab_demo.ipynb>` _.













