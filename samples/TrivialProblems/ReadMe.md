This class of problems represents the simplest possible planning problem that can be solved.
Indeed, the problems consist in connecting a pair of points in the space, finding a path that avoids some axis aligned boxes placed in the scene.

**MT-RRT** per se does not make any kind of assumption on the particular problem to solve. 
Indeed, all the logics and knowledge specific to be problem, should be contained in a **Connector** kind object, which is at a later stage consumed by **MT-RRT**.
To this purpose, the **TrivialProblemConnector** defined [here](src/) extends the **Connector** base and is used by the samples.

The samples are contained [here](./scenarios). Each sample, is an executable consuming both **MT-RRT** as well as **TrivialProblemConnector**.
Each scenario:
    - has some default configurations to set up **MT-RRT** (the kind of solver to use, the parameters to configure the solver, etc...)
    - possibly receive from the user a **.json** to completely/partially override the above configurations. The functionalities used to parse such **.json** are defined [here](../configurations/)
    - solve the specific problem with the configured solver and store the relevant data into a **.json** output
    - the results contained in the generated **.json** can be then visualized using a python script which produces nice plottings (the specific invokation of such script is suggested by the scenario itself)
