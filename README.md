# BIS

[<img src="https://img.shields.io/badge/docs-latest-blue">](https://wei-tianhao.github.io/BIS/build/html/index.html)
[<img src="https://img.shields.io/badge/version-1.0-brightgreen">]()

### Introduction

BIS is an open source benchmark for safe control algorithms. 

You can evaluate your algorithm on various robot models, including ball model, unicycle model, robot arm model, etc. 

We also provide comparison with different algorithms. Currently implemented algorithms including Potential Field method, Safe Set algorithm, Sliding Model algorithm, Barrier Function algorithm, and Sublevel Safe Set algorithm. 

Various metrics are provided. Numerical analysis like Safety-Efficiency trade-off curves and Speed profile.

![passive_results](docs/images/passive_results.jpg)

We also provide visual comparison tools.

![visual_comparison](docs/images/visual_comparison.jpg)

*We used a fancy model to draw paper graphs. But due to the complex appearance and large volume of the fancy model, it may cause precession problem in the actual test. So, by default, we only use simple models like cube and ball to describe the robot.

### Document

Detailed documents of the code can be found at <https://wei-tianhao.github.io/BIS/>.

### Install

To install this repository:

```bash
cd YOURPATH
git clone git@github.com:intelligent-control-lab/BIS.git
#create a virtual environment named psb_env, using the latest python3.6
conda create -n BIS python=3.6
#activate the virtual env
source activate BIS
#install requirement packages
pip install -r requirements.txt
#The next step is only required for MacOS:
#to use matplotlib.pyplot in a conda virtual environment, we need to install python as a framework
conda install python.app
```
### Tutorial

To add new algorithm and new model, please refer to tutorial.ipynb

To get the results on our paper, please run

```bash
python leaderboard.py
```

To see a demo, please run
```bash
python test.py
```

