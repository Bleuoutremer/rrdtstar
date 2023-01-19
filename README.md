
Former work [paper](https://arxiv.org/abs/1810.03749) from ICRA'19. 

```sh
# assuming python3 and bash shell
python -m venv rrdtstar
source rrdtstar/bin/activate
```

```sh
pip install -r requirements.txt
```


detailed help message with
```sh
python main.py --help
```

basic syntax is
```sh
python main.py <PLANNER> <MAP> [options]
```

examples
```sh
python main.py rrdtstar maps/room1.png -vv
```

<p align="center">
    <img width="600" height="600" src="doc/images/rrdtstar.gif" alt="rrdtstar Planner" />
</p>

compare with rrtstar

<p align="center">
    <img width="600" height="600" src="doc/images/rrtstar.gif" alt="rrtstar Planner" />
</p>

### RRT*

```sh
python main.py rrt maps/room1.png -vv
```

<p align="center">
    <img width="300" height="300" src="doc/images/rrt.gif" alt="RRT* Planner" />
</p>

### Bi-RRT*

```sh
python main.py birrt maps/room1.png -vv
```

<p align="center">
    <img width="300" height="300" src="doc/images/birrt.gif" alt="Bi-RRT* Planner" />
</p>

### Informed RRT*

```sh
python main.py informedrrt maps/room1.png -vv
```

<p align="center">
<img width="600" height="auto" src="doc/images/informedrrt.gif" alt="Informed RRT* Planner" />
</p>

The red ellipse shown is the dynamic sampling area for Informed RRT*


