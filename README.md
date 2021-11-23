# agv_simulation

As all grids are only uni-direction, the layout of warehouse should satisfy some conditions. Here is an example.

## Input 

- height 12

- width 12

- agents

| x | y | agent_type | n |
| :---: | :----: | :---: | :---: |
| 4 | 7 | workstation | |
| 7 | 1 | workstation | |
| 1 | 4 | workstation | |
| 1 | 7 | workstation | |
| 4 | 10 | workstation | |
| 7 | 10 | workstation | |
| 10 | 4 | workstation | |
| 10 | 7 | workstation | |
| 4 | 4 | dropstation | |
| 4 | 7 | dropstation | |
| 7 | 4 | dropstation | |
| 7 | 7 | dropstation | |
| 1 | 1 | agv | 1 |
| 1 | 10 | agv | 1 |
| 10 | 1 | agv | 1 |
| 10 | 10 | agv | 1 |

- packages

| orig | dest | agv | 
| :---: | :----: | :---: | 
| 1 | 1 | 0 |
| 1 | 1 | 0 |
| 1 | 1 | 0 |
| 1 | 1 | 1 |
| 1 | 1 | 1 |
| 1 | 1 | 1 |
| 3 | 2 | 2 |
| 3 | 2 | 2 |
| 3 | 2 | 2 |
| 3 | 2 | 3 |
| 3 | 2 | 3 |
| 3 | 2 | 3 |

- dir_path directory for visualization

## Output

an animation is generated in the corresponding directory

## Command

`python main.py 12 12 ../test/input/agents.csv ../test/input/packages.csv ../test/fig/`