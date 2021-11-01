# Introduction
[MuJoCo](https://mujoco.org/) stands for Multi-Joint dynamics with Contact. It is a physics engine for simulation and has been open source. To get more details, see these repositories: [MuJoCo](https://github.com/deepmind/mujoco) and [MuJuCo-py](https://github.com/openai/mujoco-py).

In this repo, some mujoco example scripts and the xml files of different Unitree robots are provided, including A1, Aliengo, Laikago and Go1. To simulate different robots, the corresponding xml files are needed. Here we introduce how to generate xml files from urdf and how to use them.

# Generate XML
In MuJoCo, robots are described by xml files. Before we generate the xml file, we need the urdf file of the robot. There are many ways to obtain urdf, and these methods are not discussed here. After generating the urdf, you must ensure that all meshes are stl files. If you are using meshes in obj or other formats, convert them to stl. 

## Edit URDF
Before converting urdf to xml, some parts of urdf file need to be edited. Take "/data/urdf/a1.urdf" as an example.

1. Add MuJoCo tag
   
   Add the following line after the robot description line:
   ```
    <mujoco>
            <compiler 
            meshdir="../meshes/" 
            balanceinertia="true" 
            discardvisual="false" />
            <option gravity='0 0 -9.8'/>
    </mujoco>
   ```

2. Add a floating joint between robot body and world frame
   
   Since MuJoCo set the base link as fixed by default, a free joint has to be manually added between robot body and world frame:
   ```
    <link name="world"/>
    <joint name="floating_base" type="floating">
        <origin rpy="0 0 0" xyz="0 0 0"/>
        <parent link="world"/>
        <child link="trunk"/>
    </joint>
    ```

## Generate xml
MuJoCo provides an executable file for converting urdf to XML. This file is under "/mujoco/bin". To use it:
```
./compile /path/to/model.urdf /path/to/model.xml
```

## Edit xml file
After converting the urdf to xml, some parts of xml file need to be edited.

1. Add ground and light source:

Add these line after "worldbody" tag:
```
<light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>    
<geom type="plane" pos="0 0 -0.45" size="10 10 0.1" rgba=".9 0 0 1"/>
    
```

2. Remove the visibility of the collision parts:

In the model, some parts are used as collision detection. For these parts we do not want them to show in the simulation. To make them invisible, we need to edit the rgba value.

For instance, to hide the body collision part, just set the last number of rgba to 0:
```
from
<geom size="0.1335 0.097 0.057" type="box" rgba="0.913725 0.913725 0.847059 1" />
to
<geom size="0.1335 0.097 0.057" type="box" rgba="0.913725 0.913725 0.847059 0" />
```

For now, all the preparation is done and we can simulate our robot in MuJoCo.

# Simulation