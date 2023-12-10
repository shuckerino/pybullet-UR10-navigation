# Project Name
![](https://github.com/TristanBester/pybullet-UR10-navigation/blob/main/assets/demo_angle_two.gif)

![](https://github.com/TristanBester/pybullet-UR10-navigation/blob/main/assets/demo_angle_one.gif)

## Ovierview
This project demonstrates how to control a Universal Robots UR10 robot in the PyBullet physics simulator using Python. PyBullet is a physics engine that allows for simulation and control of robotic systems. The UR10 is a popular industrial robot with six degrees of freedom.

## Prerequisites
- Python: 3.11+
- Poetry: 
```bash
curl -sSL https://install.python-poetry.org | python3 -
```
- FFmpeg (for video recording): 
```bash
sudo apt-get update && sudo apt-get install -y ffmpeg
```

## Getting Started
Clone the repository:

```bash
git clone https://github.com/your-username/ur10-pybullet-control.git
cd ur10-pybullet-control
``` 

Install dependencies:

```bash
poetry install
```

Run the simulation:

```bash
cd src && poetry run main.py
```


