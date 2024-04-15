# Manipulators

This repository contains the code related to the **"Modeling and control of robotic manipulators"** course. This considers the simulation through the [PyBullet](https://pybullet.org/wordpress/) environment of 2 robots: <br>

- 6 DoF Elbow robot with spherical wrist<br>
![Screenshot from 2022-10-31 13-23-26](https://user-images.githubusercontent.com/107052856/199092839-73d66259-aa06-4ae8-82e2-28cc14799f0a.png) <br>

- 4 DoF Scara robot  <br>
![Screenshot from 2022-10-31 13-24-47](https://user-images.githubusercontent.com/107052856/199092952-69f87506-d605-4249-bea1-e5a1c9cc3964.png) <br>

# Contents
- Unified Robot Description Format (URDF)
- Visual (dae) and collision meshes (stl)
- Mathematical models necesary for the drivers
- Forward kinematics solver
- Inverse kinematics solver
- Third order trajectory generator
- Simulation scripts
- CAD files in [FreeCad](https://www.freecadweb.org/) format <br>

![videoplayback_AdobeExpress](https://user-images.githubusercontent.com/107052856/199100621-44db1fd3-5e2c-4b89-8b7e-ad8c024bcd9d.gif)
![ezgif-2-ad0853bce6](https://github.com/istarendil/manipulators/assets/107052856/9f1c690b-dcd7-4168-a991-ea367487c325)
![output_page-0001](https://user-images.githubusercontent.com/107052856/199103282-c0a121b3-920c-4775-8431-27d99d2e0f1f.jpg)

# Tools
|Name|Version|
|----|-------|
|Python|3.10|
|PyBullet|3.2.5|
|Numpy|1.23|

# Demo
- 6 DoF Elbow robot [Elbow_trajectory](https://www.youtube.com/watch?v=byPdZVqNF7Y&list=PLQBwkbxMqU0CwwgrcaWHP4ouFjho0Iy4H&index=11) without considering the collision meshes
- 6 DoF Elbow robot [Elbow_collision](https://www.youtube.com/watch?v=Ua7ChVYoCIc&list=PLQBwkbxMqU0CwwgrcaWHP4ouFjho0Iy4H&index=12) considering the collision meshes
- 4 DoF Scara robot [Scara_P&P](https://youtu.be/VX4Uf92NGkc?si=_Ugo5zqfH_W6do_c) during a pick & place application
