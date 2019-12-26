# Robotics Software Projects
This repo is where robotics projects and some Udacity Robotics projects are kept as a reference for the future.

## Gazebo
_**7~7.X**_
This is how to upgrade your Gazebo7 to Gazebo7.X  
Where X means the latest version of 7  
All you need to do is just add the repositories and key as described in [link](http://gazebosim.org/tutorials?cat=install&tut=install_ubuntu&ver=7.0)  
```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```

And then run the installation command  
`sudo apt-get install gazebo7`  
`sudo apt-get install libgazebo7-dev`  

To check if you have the latest version of gazebo, run the following command  
`gazebo --version`

_**7~9**_
This is how to upgrade your Gazebo7 to Gazebo9  
First, remove Gazebo7(2 Steps)  
```
sudo apt-get remove ros-kinetic-gazebo-*
ros-kinetic-gazebo-dev
ros-kinetic-gazebo-msgs
ros-kinetic-gazebo-plugins
ros-kinetic-gazebo-ros
ros-kinetic-gazebo-ros-control
ros-kinetic-gazebo-ros-pkgs
```
The printout
```
Removing ros-kinetic-pr2-desktop (1.1.3-0xenial-20191127-173214+0000) ...
Removing ros-kinetic-desktop-full (1.3.2-0xenial-20191127-163953+0000) ...
Removing ros-kinetic-simulators (1.3.2-0xenial-20191127-162721+0000) ...
Removing ros-kinetic-gazebo-ros-pkgs (2.5.19-1xenial-20191119-092329+0000) ...
Removing ros-kinetic-pr2-simulator (2.0.11-0xenial-20191127-172621+0000) ...
Removing ros-kinetic-pr2-gazebo (2.0.11-0xenial-20191127-171548+0000) ...
Removing ros-kinetic-pr2-gazebo-plugins (2.0.11-0xenial-20191119-054928+0000) ...
Removing ros-kinetic-message-relay (0.0.2-1xenial-20191119-061146+0000) ...
Removing ros-kinetic-kobuki-gazebo-plugins (0.5.7-0xenial-20191119-054954+0000) ...
Removing ros-kinetic-gazebo-plugins (2.5.19-1xenial-20191119-053038+0000) ...
Removing ros-kinetic-gazebo-ros-control (2.5.19-1xenial-20191127-163246+0000) ...
Removing ros-kinetic-gazebo-ros (2.5.19-1xenial-20191119-053032+0000) ...
Removing ros-kinetic-gazebo-dev (2.5.19-1xenial-20190607-165824-0800) ...
Removing ros-kinetic-gazebo-msgs (2.5.19-1xenial-20191119-052225+0000) ...
```
Second, remove Gazebo7 manually  
```
sudo apt-get remove libgazebo*
sudo apt-get remove gazebo*
```

Install Gazebo9
`sudo apt-get install ros-kinetic-gazebo9*`

That should do the trick.
[Reference1](https://medium.com/@abhiksingla10/setting-up-ros-kinetic-and-gazebo-8-or-9-70f2231af21a)
[Reference2](https://community.gazebosim.org/t/all-about-gazebo-9-with-ros/187)

# Others
or create a new repository on the command line

echo "# RoboticsSoftwareProjects" >> README.md

git init

git add README.md

git commit -m "first commit"

git remote add origin https://github.com/BruceChanJianLe/RoboticsSoftwareProjects.git

git push -u origin master

or push an existing repository from the command line

git remote add origin https://github.com/BruceChanJianLe/RoboticsSoftwareProjects.git

git push -u origin master

or import code from another repository

You can initialize this repository with code from a Subversion, Mercurial, or TFS project.
