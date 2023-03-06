### ROS2 over WiFi

In general, any active ros nodes must be accessible by any device on the same WiFi network.  
But in practice, the topics are many times not seen by ros2 on other machines despite being on the same network.  

The major problem is Firewall enabled in the network  
To disable the same, in terminal of eavery machine use  
`sudo ufw disable`   
This disables the firewall and then allows the ros nodes to access all topics on the network. 

This practice is not that safe since we need a way to allow only the machines we need on the network to have access rather than all machines on the network
THis can be done by setting rules for disabling the firewall for certain IP addresses  
`sudo ufw allow in proto udp from 192.168.1.0/24`

Using this command on both machines will allow the ros nodes to see each others topics provided they are using the specific port.
  
  
  

Sources: 
 [1] https://stackoverflow.com/questions/75006253/ros2-on-multiple-machines-ros2-multicast-working-talker-listener-not-working
