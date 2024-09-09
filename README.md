# pronto_estimator_ws
this repositories contains the ros2 humble workspace to use Pronto EKF estimator and eventually access to the qualysis ground truth data. The whole workspace has been dockerized to enhance the code portability.
## prerequisite 
TODO for no docker version 

## installation

First of all clone this repository, then initialize the git submodule inside the src directory.

``` cd pronto_estimator_ws ```

``` git submodule update --init --recursive ```

Then you have to build the docker image:

``` bash build.bash ```

At the end to run the Docker container:

``` bash run.bash ```

## usage

First you have to add to the workspace the robot descrition package cointain the urdf of the robot, then compile the workspace again.

Starts your robot platform and load the controller to get the robot's sensors information.

If it is needed start the qualysis system, then set up the filter cofiguration, adding a new yaml file in order to preserve the older config file, and then start the pronto estimator node.

