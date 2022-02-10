# Vehicule properties

## Acceleration profile


## Speed stability 

### Treshold algorithm with 0.5 limit

| Speed       | Trajectory    | Speed stability|
| :---        |    :----:     |          ---:  |
| 20          | Straight line | 0.007          |
| 50          | Straight line | 0.005          |

### Treshold algorithm with 1 limit

| Speed       | Trajectory    | Speed stability|
| :---        |    :----:     |          ---:  |
| 100         | Straight line | 0.018          |
| 50          | Straight line | 0.023          |


## Response time

| Parametes (Kd, Ki, Kp)   | Target speed  | Response time  |
| :---                     |    :----:     |          ---:  |
| (0, 0, 5)                | 50            | 4.9            |
| (0, 0, 5)                | 100           | 10.6           |
| (0, 0, 5)                | 100           |                |


## Turn profile

For each angle, we look for the speed at which it starts skiding.

| Angle | Max speed |
|  :--- |  :---     |
|  0.2  |  75.5     |
|  0.3  |  60.3     |
|  0.4  |  51.8     |
|  0.5  |  46       |
|  0.6  |  42.4     |
|  0.7  |  39.6     |
|  0.8  |  37.0     |
|  0.9  |  35.0     |
|  1.0  |  33.5     |
