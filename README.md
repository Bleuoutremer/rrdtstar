
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

<table align="center">
    <tr>
        <td>
            <img width="300" height="300" src="doc/images/path_planning.gif" alt="rrdtinformed Planner 1" />
        </td>
        <td>
            <img width="300" height="300" src="doc/images/rrdtinformed.gif" alt="rrdtinformed Planner 2" />
        </td>
    </tr>
</table>


compare with rrtstar （left: my method; right: rrtstar)


<table align="center">
    <tr>
        <td>
            <img width="300" height="300" src="doc/images/rrdtinformed.gif" alt="rrdtinformed Planner 2" />
        </td>
        <td>
            <img width="300" height="300" src="doc/images/rrtstar.gif" alt="rrtstar Planner" />
        </td>
    </tr>
</table>


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



