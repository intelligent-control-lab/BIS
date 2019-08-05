# BIS

BIS is an open source benchmark program for evaluating safe control algorithms.

```bash
#create a virtual environment named psb_env, using the latest python3.6
conda create -n BIS python=3.6
#activate the virtual env
source activate BIS
#install panda3d, this is a 3d game engine we use to render graphics
pip install panda3d==1.10
pip install cvxopt
#to use matplotlib.pyplot in a conda virtual environment, we need to install python as a framework
#(only required for MacOS)
conda install python.app
#to convert the figure to latex code, we need this lib
pip install matplotlib2tikz
```
To get the results on our paper, please run
```bash
python leaderboard.py
```

To see a demo, please run
```bash
python test.py
```
