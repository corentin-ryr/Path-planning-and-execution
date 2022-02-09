# Path planning and execution

A car is guided through a maze by a path-finding algorithm and further post-processing to enhance the car trajectory. We control the car with a Pure Pursuit algorithm.


## Path travel time estimation

We measured the properties of the car (aceleration max speed in curves).
From the path curvature and length, we computed the theoretical speed (first the max speed at the path curvature and then we took into account the acceleration).

On terrain C (the simplest one) and A the estimation was half of the real time. This is due to the wrong estimation of the beginning of the vehicule (we can expect the error to be a fixed error, the error percentage is big here because of the short path) and probably due to the speed being in miles per hours.

## References

<a id="1">[1]</a> P. E. Hart, N. J. Nilsson and B. Raphael, "A Formal Basis for the Heuristic Determination of Minimum Cost Paths," in IEEE Transactions on Systems Science and Cybernetics, vol. 4, no. 2, pp. 100-107, July 1968, doi: 10.1109/TSSC.1968.300136.

<a id="2">[2]</a> Coulter, R. Implementation of the Pure Pursuit Path Tracking Algorithm. Carnegie Mellon University, Pittsburgh, Pennsylvania, Jan 1990.

<a id="3">[3]</a> Hoffmann, Gabriel M., Claire J. Tomlin, Michael Montemerlo, and Sebastian Thrun. "Autonomous Automobile Trajectory Tracking for Off-Road Driving: Controller Design, Experimental Validation and Racing." American Control Conference. 2007, pp. 2296–2301. doi:10.1109/ACC.2007.4282788 (Stanley Controller)

<a id="4">[4]</a> Xiao-Diao Chen, Yin Zhou, Zhenyu Shu, Hua Su, Jean-Claude Paul. Improved Algebraic Algorithm On Point Projection For Bézier Curves. (Not used yet)