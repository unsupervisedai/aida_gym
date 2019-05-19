# How to modify the urdf folder

This short tutorial doesn't aim to explain you everything you can find in these files, but only the basic material required to begin.

It should be enough to create your own environnement, however if you want to discover more advanced stuff, you can find more information online :

- [ROS URDF Tutorial](http://wiki.ros.org/urdf/Tutorials)
- [.obj files Wiki page](https://en.wikipedia.org/wiki/Wavefront_.obj_file)
- [.mtl files Wiki page](https://en.wikipedia.org/wiki/Wavefront_.obj_file#Material_template_library)

The urdf folder contains all the files describing physically the simulation's environnement.
The folder "meshes" + the file "aida.urdf" represents the robot Aida.
The other files are for the environnement in which Aida will walk.

There are basically 3 types of files : .mtl, .obj, .urdf

## .mtl files

A .mtl file describes a material. Each line (name + value) is a parameter (illumination, reflection...) which change the way you see the material.

The most useful to begin is 'map_Kd' which allow you to load an image on your material :

```
map_Kd checker_blue.png
```

## .obj files

A .obj file describes an object. First you give the name of your object :

```
o Plane
```

Then, the shape of the object is given by its vertices (v) in cartesian coordinates (x,y,z) :

```
v 50.000000 -50.000000 0.000000
v 50.000000 50.000000 0.000000
v -50.000000 50.000000 0.000000
v -50.000000  -50.000000 0.000000
```

You can also specify texture coordinates with 'vt' and normal coordinates with 'vn'. For more details about how the texture is placed on the faces, search "UV Mapping".

Each element is here indexed (beginning at 1) in the order they are written. For example, here v2 = {50.000000 50.000000 0.000000}. It is the same with the vt and the vn.


Then, we form the triangle faces of the object by giving group of 3 points with "f v1 v2 v3" :

```
f 1 2 3
```

This will form a triangle between the points 1, 2 and 3. Warning, the order given is important because the faces are oriented (rotating right-hand rule). Hence, the faces 1 2 3 and 1 3 2 are not the same.

If you want to add textures or normals, use the format :

```
f v1/vt1/vn1 v2/vt2/vn2 v3/vt3/vn3
```

Finally, if you want to add texture materials, you need to load them :

```
mtllib plane.mtl
```

and declare each material before the faces concerned :

```
usemtl Material
```

with "Material" being the name of the material given in the .mtl file.

## .urdf files

A .urdf file describes a robot, using .obj objects. Here, the ground is considered as a static robot.
A robot is given by links (pieces of the robot) which are joint together.

There are many parameters in a link. The most important are the "visual" (what it looks like) and the "collision" (how it interacts with the rest).
In these parameters, you specify an "origin" and a "geometry" (that's where you can load .obj files).

```
<geometry>
	<mesh filename="plane.obj" scale="1 1 1"/>
</geometry>
```

Other parameters are necessary to launch the simulation, but you do not need to change them to begin.

Then you compulsorily need to join your links together :

```
<joint name="joint" type="fixed">
    <parent link="Link1"/>
    <child link="Link2"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
</joint>
```

The type of the joint (fixed, revolute, floating ...) determines the way your pieces will interact together.

All the pieces (links) together will form the robot. In aida simulation, the urdf file loaded as a ground is specified in the file "aida_gym_env.py". It is now a parameter you can give when calling the simulation :

```
env = e.AidaBulletEnv(render=True, area = "marche", on_rack=True, commands = [...])
```

This will load marche.urdf as the ground, default being plane.urdf.

## How to begin ?

There are currently 3 environnements available :

- plane.urdf is a basic flat surface without any obstacle

- marche.urdf is the same flat surface with a green step in front of Aida

- terrain1.urdf is an uneven ground composed of many small steps


The files for the "marche" were written by hand using the "plane" as a basis.

The file for "terrain1" (including the file bords.obj) were written thanks to generating Python scripts (terrain1.py and terrain1-2.py).

A good way to start is to try to understand the files for the plane and the marche, changing parameters. After that, try to understand the Python scripts to generate the "terrain1". Once you're done, you should be able to write your own environnement easily.








