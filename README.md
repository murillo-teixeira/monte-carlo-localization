# Monte Carlo Localization

## Instructions

1. Create a catkin workspace that will contain the code inside the home

```bash
cd ~
mkdir -p catkin_ws/src
cd catkin_ws
catkin build
```

2. Clone this repository inside the catkin_ws/src folder

```bash
cd src
git clone https://github.com/mhteixeira/monte-carlo-localization.git
```

3. Build the package

```bash
cd ~/catkin_ws
catkin build
```

4. Include the following line inside the ~/.bashrc file (using e.g. 'nano ~/.bashrc')

``` bash 
source $HOME/catkin_ws/devel/setup.bash
```

and then do:

``` bash 
source ~/.bashrc
```

4. Run the launch file

``` bash 
roslaunch mcl_python load_mcl.launch
```