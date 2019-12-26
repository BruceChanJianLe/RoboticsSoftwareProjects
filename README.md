# Robotics Software Projects
This repo is where robotics projects and some Udacity Robotics projects are kept as a reference for the future.

## Gazebo

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

###

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
