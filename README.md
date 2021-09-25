## FCND-Term1-P2-3D-Motion-Planning
![Drone Flying](./images/drone_flying.gif)
This project is part of [Udacity](https://www.udacity.com "Udacity - Be in demand")'s [Flying Car Nanodegree](https://www.udacity.com/course/flying-car-nanodegree--nd787).  The project is to integrate the technique, such as A Star Search and Voronoi Map, that learned throughout the last several lessons to plan a 3D path through an urban environment.   The python code communicates with Udacity FCND Simulator using Udacidrone API on Mac Mini 1.  The project is using Miniconda with Python 3.6.1


---


# Project Description
The project mainly to develop a class method named plan_path() to execute the action below:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* , Voronoi or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates.

## Implementation
The motion planning is built mainly relied on the algorithm of A* Search.  However, I also provided an alternatively method, that is Voronoi Diagram.  The motion path built by A* Search usually get the shorter path.  However, the motion path got from Voronoi diagram is usually safer because the path is always at the middle of the obstacles.  The following is the diagrams for the motion path that planned under the algorithm of A* Search and Voronoi Diagram respectively.
![A* Search](./images/A Star Motion Path Graph.png)

![Voronoi Diagram](https://lh3.googleusercontent.com/O54EMbZvhGHGAY5As4gR1CMBEtevEJMlQiM0tazjdLKOZvFNu_1UZjydYCDuNgOdX14KivK8BrWuoSO3hN5AA0ZbKV8QNpcRTd-MNgEZmO2EsvowJxcIawrqjAremLs71fOmL49B4Y4D99QeVwKda0EvyU4kt1jL9sSZT_W51GWdSfTJ4fpwqix0puM34hXioW6mOpc_3NweuKTYlwqmqplLguNUIuewi4sCf5S1QXENKViWnE8DHGESdpvuegfnzbIxXvSkHLapHTrO3vt2JFmXoQkGcJBMZy7z9L1-Nu6vXdtAaq6ZC2wWDift7kNRsVRVhr5_W7CKHmzB-duwD5XT4Eu3xpQxRmpwz5j82el_LGyAvqQjV-Zh8p-ZyFVs_RatgXlq40TX9iLbSQkmVJrGEuhm1SnD0cMMnZtqQsybQ1W_nFeqo7nLdh8zf7ux5LHOFt1tQakhxOn-HdfiJzy-9DRmRL3ZUFMy4GyJyGoLaoZJ9Akme-EfwXLh_fEpTyFKDFGSKUGYgeLTvIpm4bO0RqtCauocIqZ9Imy8TJQb9tfzXASwdfxFuFax9_aumpO8daFCaJ-rU9YWjn9Oz5RRPpTLOE8wJWI5DtxweD9ihbLIP4JryMRQZDwaenXX6PMrZEr3CeRMxNEdhNDh6wznMxSD-Sfyha07nfShQhKYeep3RAFeIXz2UYopLLz5HjOqMBqCyZCohMok3lFNVzq-=s864-no?authuser=0)



## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points



### Explain the Starter Code

#### The starter code contains two files :`motion_planning.py` and planning_utils.py
The script of motion_planning.py contains a basic planning implementation that includes the setup of states for drone and callback routines for the operation handshaking between the drone and the computer.
![Operation Handshaking](https://lh3.googleusercontent.com/Wd0iZqlxMshO2IvRMbQUxkbgmJLckhTdLCOwSjwOoXkyA5ImGiZCVN2ZpsfR2uMoNiGRMzRHMk1ShS2QwcZr4l5QE4dcSOLsWsiwJAOpRAPmW8-KhMOJ0Q7djeb4FQlbw6PyZcatDAQkvcuL2EyTfxklQl8ORo68gdHiUcKmr682QDmmSejpfbW-H-dFDx6FPIEodf88J-IFLuUd7dBjd2kb3GLDfnS6DR7qy8_7qVDGIhQFN53XMGh64-WMhImFO7IhgVjClmVXUgd9i33pjkgACcCGbyaycn6su8fZsxYdAHG9c1gaBdJMZHWFsZgj3e9JKxNRlrTAbO5ItY3kQoC6-gdnJBRZFeJt7ctiHd_J2LGFXyOFSxsYdWIGMmeD8tnIqzoWN20yZqaLNR76-q4p-RghgJEkKTaMBPm6IgcdJNHOlI521kXg6ZAbRYFTirnyzRAPoyRr_JVJ3KddQpLBbF9Tj2WKgKAc178ygrXNT1cok9IlvdOJQsb1aDC3cB_s0d2AZrp0cc4mauOV3D8MEA8SC5xm3U1iYMUMWnzfxb9YlThHQb3h2hvTNHK4Dz_5WEgZBEhDhtAiCYnd6n1YWUzz6i3iMjvkmKzhbYWLxNwAR3-yFhcWgrhIXy6zzvGMbKiF4K_FoU7uBWdDM1F4Ty20kxJ1P5CKWLrNRHfBnld-wOjvbWj7qye-TM1p4HFtiASdQu3Z6ZWjUFyb4ZgO=w960-h720-no?authuser=0)


The script of planning_utils.py provides the utilities of grid creation, A Star Research and also some prune utilities like collinearity.

### Implementing Path Planning Algorithm

#### 1. Set the global home position
We need to read the first line of the csv file, extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home. 
			
<font size="2"> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;F1 = open('colliders.csv', 'r')
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;S1 = F1.readline(-1)
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;S2, S3 = S1.split(", ")
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;_, S4 = S2.split()
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; _, S5 = S3.split()
            &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;lat0 = float(S4)
            &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;lon0 = float(S5)
            &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;self.set_home_position(lon0, lat0, 0)
</font>


#### 2. Set the current local position
We then can get the global current position of the drone from the class variable: _longitude, _latitude and _altitude.  With the help of function global_to_local(),  we can easily convert the global current position to current local position.
<font size="2"> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; global_pos_current= [self._longitude, self._latitude, self._altitude]
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; local_pos_current= global_to_local(global_pos_current, self.global_home)
</font>

#### 3. Set grid start position from local position
We then create a grid with the help of create_grid_and_edges() function.  The grid start can be any point inside the grid but cannot be inside the obstacles.
<font size="2"> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;grid, edges, north_offset, east_offset = create_grid_and_edges(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; grid_start= (int(np.ceil(self.local_position[0] - north_offset)), int(np.ceil((self.local_position[1] - east_offset))))
</font>

#### 4. Set grid goal position from geodetic coords
The desired goal location should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.  Of course, the goal position cannot be inside the obstacles.
<font size="2"> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; grid_goal= (int(np.ceil(-north_offset + 500)), int(np.ceil(-east_offset + 300)))
</font>
#### 5. Modify A* to include diagonal motion (or replace A* altogether)
To include the diagonal motion in A* search,  beside NORTH,  EAST,  SOUTH and NORTH, we need to add four more orientations that is NORTH_EAST, SOUTH_EAST, SOUTH_WEST and NORTH_EAST.
<font size="2"> 
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;NORTH_WEST = (-1, -1, sqrt(2))
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;NORTH_EAST = (-1, 1, sqrt(2))
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;SOUTH_EAST = (1, 1, sqrt(2))
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;SOUTH_WEST = (1, -1, sqrt(2))
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;						    :
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;						    :
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;#SOUTH_EAST		
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;    if x + 1 > n or y + 1 > m or grid[x+1, y+1] = = 1:
        &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;valid_actions.remove(Action.SOUTH_EAST)
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;#NORTH_EAST
   &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; if x - 1 < 0 or y + 1 > m or grid[x-1, y+1] = = 1:
        &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;valid_actions.remove(Action.NORTH_EAST)			
   &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; #SOUTH_WEST
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;if x + 1 > n or y - 1 < 0 or grid[x+1, y-1] = = 1:
        &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;valid_actions.remove(Action.SOUTH_WEST)
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;#NORTH_WEST
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;if x - 1 < 0 or y - 1 < 0 or grid[x-1, y-1] = = 1:
       &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;valid_actions.remove(Action.NORTH_WEST)	
</font>        


#### 6. Cull waypoints 
To prune the path of unnecessary waypoints, here we use the ray tracing method -- Bresenham.
<font size="2">     &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; pruned_path = [p for p in path]
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; i = 0
   &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; while i < len(pruned_path) - 2:
       &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; p1 = pruned_path[i]
       &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; p2 = pruned_path[i + 1]
       &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; p3 = pruned_path[i + 2]
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;  if  all((grid[cell] == 0) for cell in bresenham(int(p1[0]), int(p1[1]), int(p3[0]), int(p3[1]))):
            &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;pruned_path.remove(p2)
        &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;else:
            &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;i += 1      
</font>

![A* Search Pruned Motion Path](https://lh3.googleusercontent.com/QD78TmY0rmL_Syck3lowSxk8sLIShFtUUrdhlEYgrLFijH3zZvv82h45xOZApMbM_elhJcri1AvTdMRmCzM8BTD8A8F3gyTe2ryx4EfUPhve_n4C-L7PIAtRL2PtoG50q_iUO9ia9wNA_li1PeR4cEujyqTKZabUV7K5LO9ahZkoSzfH1TdPFR58U4XGWyLiUutldzld8C9fMVW_MKEvgjYldw_5JYCfGJ7f9SzF8tuYXC55mVnS-gaBQvLWLZkpf1HEbJ3uBbZu7AtMvou7nwBtaCcRVDQknL6ajwR_T2Q0PQF45XR0zB6vCLYJOZ-NuYzyWCl23xER6LOKJk55clB0aN_eYhRYUlYTvRU5M-T8Samr9wQ_RV5e_t4P8jVco63uaUWEHLqI9UShA-BSXyIetM1X6N_TbuOX1Ys5aeENWTFL6HgPtma4VAegChVyDAGqIW-iAaF5jkLJPhA2r9IiXXQ_hNiOoEREVu_UuV-7lMGQffG3fmAK-JJZPDBZhvXfdNXjAI-UzePSvZqNNo0jMyZR59OCMtT6-adtsPM_xct4oXsk77zO29z0sivrq0Lca9gjoz8Zc8HFBIMInDRgQ_yeNVDiWpg2pGEPF-MCmulYD_bzLclHEx2LftKXdLpXsbyy6ATHxF_nbs9aGSLjrab9J7iu6bAiUvzSHYkWVj8EJtuhG5gYkMNVqarBoUrL2gKVHqSLGCjVccQOB2YK=s864-no?authuser=0)

![Pruned Motion Plan from Voronoi Diagram](https://lh3.googleusercontent.com/Djr8kKsN5tduaHx_02lH7yvHlhBuMRXhcZy98G9fA_91DStMpkGsUrA5nDPxhf9CmSdP1g_VwBH3dSWT1yREsbaUQvSpj5Iyd90tQEo-B0cCNw75yDPBFjpRI_eSTXqRSZFUa_PNd-6X9G_1i8WIkx0gof-kzCePUuV1TovjcWomZTzneBTrnXDXldDSzPUcYGs_WpDFJULzATuoxBMUE1faxK2cgIz2CctvQQ0mdFq_61WElf9OgpzFbFql2_IC7iq3-1TyyiNLWA3sbP2aRMeOdFIrZ1P1uwUYvCFxT6cBXAeZDu9oZgHBCMN610u8ttQFfT8lkhjrDos1mdBJYzmurb3KfYoYDA6QL3gQMWBhympPeh002T5UsOUMvOTECG-k7iChfMS0vejQnohNFV8joWKj8_Yj6VtJo2DketIpOwxHTHDbAR72Ipj4a_t8ee9ebOG65-4zjZ0ahmqSHMHoSKVWNfEoCp6WD8ldetjO0FsYgOsmzXtVRVpSkJsrx16I_bYI2lIdwz9BnDeC8MFAPhr3XR4nYbnXMRmrrtpT5_WemWo1vu8_1aA292fcyL4xPmRgoyIn5gizILISPfKIP1BmFcqlhgLJytsRzmSQuxFWgpfJUfW0_cVP9ZSDOtwQMm0uiN-VLVyop7S-_Ik8QDxShbAB5JUj5g7dZ5sX5f5wKIEFvE-ZC4WqwY26MVuluUsEPWCGJUfoKGw0Ha_5=w715-h722-no?authuser=0)
### Execute the flight
The following is the video of the Drone flying through the planned motion path with Voronoi  Diagram
[![Motion Planning](http://img.youtube.com/vi/gVI1KYsm3mc/0.jpg)](https://youtu.be/gVI1KYsm3mc)

### Conclusion
This is a very challenge topic.  Currently,  I just apply A* Search and Voronoi Diagram on the Motion Planning.  The result is very good.  However, such planning is lack of reaction for those ad hoc case, such as some moving objects suddenly jump into the environment.  In the future, I will try to implement more real world planning such as RRT(Rapidly-Exploring Random Tree), potential field planning, receding horizon planning, in order to encounter unexpected obstacles. 
  


