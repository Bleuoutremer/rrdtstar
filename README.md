
Former work [paper](https://arxiv.org/abs/1810.03749) from ICRA'19. 

```sh
# assuming python3 and bash shell
python -m venv rrdtinformed
source rrdtinformed/bin/activate
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
python main.py rrdtinformed maps/room1.png -vv
```

<p align="center">
<video width="320" height="240" controls>
    <source src="doc/images/path_planning.mp4" type="video/mp4">
</video>
</p>

<p align="center">
    <img width="300" height="300" src="doc/images/rrdtinformed.gif" alt="rrdtinformed Planner" />
</p>

compare with rrtstar

<p align="center">
    <img width="300" height="300" src="doc/images/rrtstar.gif" alt="rrtstar Planner" />
</p>

### RRT*

```sh
python main.py rrt maps/room1.png -vv
```


### Bi-RRT*

```sh
python main.py birrt maps/room1.png -vv
```


### Informed RRT*

```sh
python main.py informedrrt maps/room1.png -vv
```



