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

1. Set environment

Add the following line after " size njmax="500" nconmax="100" "

```
<option gravity='0 0 -9.806' iterations='50' solver='Newton' timestep='0.002'/>

<default>
    <geom contype="1" conaffinity="1" friction="0.6 0.3 0.3" rgba="0.5 0.6 0.7 1" margin="0.001" group="0"/>

    <light castshadow="false" diffuse="1 1 1"/>
    <motor ctrlrange="-33.5 33.5" ctrllimited="true"/>
    <camera fovy="60"/>
    <joint damping="0.01" armature="0.01"/>

</default>

<asset>
    <mesh name="trunk" file="trunk.stl" />
    <mesh name="hip" file="hip.stl" />
    <mesh name="thigh_mirror" file="thigh_mirror.stl" />
    <mesh name="calf" file="calf.stl" />
    <mesh name="thigh" file="thigh.stl" />
</asset>

<asset>
    <texture type="skybox" builtin="gradient" rgb1="1.0 1.0 1.0" rgb2="1.0 1.0 1.0" width="512" height="512"/>
    <texture name="plane" type="2d" builtin="flat" rgb1="1 1 1" rgb2="1 1 1" width="512" height="512" mark="cross" markrgb="0 0 0"/>
    <material name="plane" reflectance="0.0" texture="plane" texrepeat="3 3" texuniform="true"/>
</asset>
```

2. Add ground and light source:

Add these line after "worldbody" tag:
```
<light directional="true" diffuse=".8 .8 .8" pos="0 0 10" dir="0 0 -10"/>
<camera name="track" mode="trackcom" pos="0 -1.3 1.6" xyaxes="1 0 0 0 0.707 0.707"/>
<geom name='floor' type='plane' conaffinity='1' condim='3' contype='1' rgba="0.5 0.9 0.9 0.1" material='plane' pos='0 0 0' size='0 0 1'/>
    
```

3. Remove the visibility of the collision parts:

In the model, some parts are used as collision detection. For these parts we do not want them to show in the simulation. To make them invisible, we need to edit the rgba value.

For instance, to hide the body collision part, just set the last number of rgba to 0:
```
from
<geom size="0.1335 0.097 0.057" type="box" rgba="0.913725 0.913725 0.847059 1" />
to
<geom size="0.1335 0.097 0.057" type="box" rgba="0.913725 0.913725 0.847059 0" />
```

For now, all the preparation is done and we can simulate our robot in MuJoCo.
