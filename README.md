# LightField - The Lightweight Field Robotics Visualizer  

![](https://github.com/nicrip/LightField/blob/master/doc/LightField2.gif)  

### Overview  
LightField is a lightweight field robotics visualizer written in Python and based on VTK and PyQT4. Scene objects are stored in a native QT tree structure, the QTreeWidget class. This tree structure allows transformations applied to the parent of a scene object to be propagated to its children in a structured manner.  

### Dependencies
sudo apt-get install libvtk5-qt4-dev python-vtk  
sudo apt-get install python-qt4  

### The Tree Structure  
An example tree structure is shown below.  

```
robot  
│   torso  
│   leftarm  
|   rightarm  
|   
└── head
│   │   skull  
│   │   nose  
|   |   ears
|   |
```

'Directories' allow us to group objects together, and 'objects' can be added to directories. By structuring our objects in this way, we can elegantly apply transformations to specific body parts of the robot and allow these transformations to affect the relevant children. For example, applying a rotation transform to the 'head' directory will cause the 'skull', 'nose', and 'ears' objects to rotate in sync; while applying a rotation transform to the 'leftarm' object will only cause the 'leftarm' object to rotate. Obviously, applying a translation/rotation transform to the 'robot' directory allows us to move/rotate the entire group (i.e. the whole robot) in sync. You can also apply a constant offset/orientation to an object with respect to its origin.

### Example API Usage  
You can get an idea of how to use the LightField API by examining the 'TestLightField.py' Python script. Running this script should result in an output that looks similar to below. The 'robot' in this example has the same structure as outlined above.  

![](https://github.com/nicrip/LightField/blob/master/doc/LightField1.gif)  

### Interacting with Objects  
Clicking on a scene object in the LightField Scene Manager allows you to modify its properties as demonstrated in the gif above. For example, you can display the origin of the object (the point around which the object rotates) by clicking on the Visibility box under Frame Axes. You can change the scale, alpha, color, point size, and line width of objects.  

## Work In Progress
  - LCM integration example (ExampleLCMLightField.py)
  - MOOS integration example
  - ROS integration example
  - Standard Protobuf message to add objects, apply transformations, modify objects etc.; the idea is to send this Protobuf string via whichever comms. architecture you would like (LCM, MOOS, ROS)
  - Camera focusing (center and follow a scene object)
