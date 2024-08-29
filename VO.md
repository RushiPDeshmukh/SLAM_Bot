## Visual Odometry trials

#### Jupyter notebook 
- Original method (without tf)
 ![image](https://github.com/user-attachments/assets/d2abf1bd-3828-4173-bde4-6ea9a86c665e)

- Rotation and Translation integral method implemented in notebook ( below given are maybe ss from these )
![Screenshot from 2024-08-22 17-22-26](https://github.com/user-attachments/assets/4a40ae8d-e31c-4ce2-a231-ccb4d13c5765)
![Screenshot from 2024-08-22 17-07-18](https://github.com/user-attachments/assets/d0bdc357-4961-480a-9b69-4b0f76cd924b)

#### Real time VO ( ROS Node )
- Integrating rotation and translation without filtering matches   
  Camera link has same coordinate system as base link  
  Applying camera link to camera optical link in path publishing  
        this_pose.pose.position.x = position[2]   ( X -->  Z )  
        this_pose.pose.position.y = -position[0]  ( Y --> -X )  
        this_pose.pose.position.z = -position[1]  ( Z --> -Y )  
  ![Screenshot from 2024-08-28 21-13-47](https://github.com/user-attachments/assets/57bfdd34-7042-4a48-8a80-5f937eac3e27)
 ![image](https://github.com/user-attachments/assets/97312fba-0def-4970-a613-ac1d1086c5ce)  
NOTE: This 1.5 times the path marked ( 5ft square )
