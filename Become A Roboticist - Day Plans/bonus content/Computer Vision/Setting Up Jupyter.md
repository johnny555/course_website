

Actually, copilot was pretty good here: 

It looks like you’re encountering an issue with an externally managed environment. To resolve this, you can create a virtual environment and install JupyterLab within it. Here are the steps:

1. **Create a Virtual Environment**:
    
    ```bash
    python3 -m venv myenv
    ```
    
2. **Activate the Virtual Environment**:
    
    ```bash
    source myenv/bin/activate
    ```
    
3. **Install JupyterLab**:
    
    ```bash
    pip install jupyterlab
    ```
    
4. **Install Jupyter-ROS2**:
    
    ```bash
    pip install jupyros
    jupyter labextension install jupyter-ros2
    ```
    
5. **Enable Jupyter Extensions**:
    
    ```bash
    jupyter nbextension enable --py --sys-prefix jupyros
    jupyter nbextension enable --py --sys-prefix ipywidgets
    ```
    
6. **Run JupyterLab**:
    
    ```bash
    jupyter lab
    ```
    
 although step 5 didn't work 
 
---

Ok things went alright ...

I got a simple colour segmenter. I used the depth cloud to create a point cloud in the optical frame. Then I used a python fitting library to fit a cylinder, then I jiggled with the transforms to work out how to get that cylinder to line up with the coke can.

We could plan a very simple grasp, position the gripper above the detected location, and then move down, grasp and then retract. --- it should work ok ... and would be a good demo... 

gee there have been alot of steps to get here though... 

- Setup a python env
- Start jupyter notebook
- get a node running to talk to ROS system 
- extract data from a message
- process data to find bounding box
- create a point cloud from bounding box
- fit a cylinder to point cloud 
- add cylinder to the planning scene

and now 
- grasp with a simple plan... 

Additionally, we could have it simply provide a service to run the cylinder detection pipeline. 