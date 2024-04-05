## Libraries Needed

### DepthAI and Python libraries

`sudo wget -qO- https://docs.luxonis.com/install_dependencies.sh | bash`

`python3 -m pip install depthai`

#### Setup for Laptop
1. Clone DepthAi repository from Luxonis  
   `git clone https://github.com/luxonis/depthai.git`
2. Create a virtual environment for the installation  
   `python3 -m venv myvenv`
3. Start the virtual env  
   `source myvenv/bin/activate`
4. Install dependencies in virtual env  
   `python3 install_requirements.py`
5. Run demo  
   `python3 depthai_demo.py`


#### Specs for Oak D Lite : [Oak D Lite](https://docs.luxonis.com/projects/hardware/en/latest/pages/DM9095/#dm9095)  
![image](https://github.com/RushiPDeshmukh/SLAM_Bot/assets/94715242/3f319fb6-dbab-4ad5-910b-6d233c57b40c)


References
[1] https://docs.luxonis.com/projects/api/en/latest/index.html  
[2] https://robofoundry.medium.com/oak-d-lite-camera-basic-setup-38a563cd594f  
[3] https://robofoundry.medium.com/oak-d-lite-camera-ros2-setup-1e74ed03350d  
