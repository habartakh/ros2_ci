# Continuous Integration on a ROS2 project

The goal of this project is to automate a CI process to be triggered whenever a commit is pushed or a pull request is accepted to a remote repository.

The process includes building a Docker image and starting a container that spawns the fastbot into a Gazebo world, then launch a test file to test the `fastbot_waypoints` action server. 


## Steps 

### Install Docker

First, it is extremely important to install Docker and start it on the host PC: 
```
sudo apt-get update
sudo apt-get install -y docker.io docker-compose
sudo service docker start
```

In order to use Docker without the `sudo` keyword, run: 
```
sudo usermod -aG docker $USER
newgrp docker 
```

### Start Jenkins 

It is possible to both install and/or start Jenkins using the script `start_jenkins.bash`:
```
bash start_jenkins.sh
```  
*Please note that Jenkins has already been installed on the local PC. Running the bash script above will only start it.*


In order to access the Jenkins console, refer to the `jenkins__pid__url.txt` file in `/home/user/` directory. Then, copy the URL provided in your browser.

The code to access the console are the following: 
```  
Username: admin
Password: admin
```  

Jenkins contains two projects, the one related to this repository is called `CP24_Task2`.

#### Important Note: Create a Pull Request

This repository has been setup in a way that any pull request will start a build of the jenkins workspace. 

To do that, a webhook leading to the Jenkins Console URL has been created on this Git Repo.  

However, the webhook should be modified as after each start up of Jenkins, a random URL is generated.  
Thus, please inform me of the current Jenkins URL so that I can modify the webhook in my Git Repo. 