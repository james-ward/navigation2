# Dynamic Obstacle Layer

A dynamic obstacle layer for the ROS2 Navigation stack using the costmap layer plugin interface.

Takes in messages with dynamic obstacle tracks (position, size, velocity, etc) and marks them in the costmap at the pose and size, while projecting the velocity forward in time so the robot cannot cut off moving obstacles in the local time horizon.

!!!!
NOTE TO SELF: speciality leading efforts on tools for dynamic envrionments around untrained staff. Technology and research integration into commercially usable and industrial quality systems. full stack mobile robotics expert (plan, control, nav, autonomy, hardware, architecture). Team / division leader. Idea maker. gatekeeper.
!!!!

The main concept of this is...
# explain basic marking and projection 
# explain how kinodynamic planner works and how this is analog, with 1 major known issue
# explain "cost" -> not just probability but also risk
# explain how this is a known method (fetch, rus, 2019)

# Configuration

# Assumed inputs

Filtered tracks from any number of detection methods. It can be from multiple sensors and modalities, but this layer assumes 1 input topic of tracks per update cycle so you need to merge them.

OR assumes N topics and will buffer and use them all, but its assumed that if there's any FOV overlap that you do merging of the dynamic obstalc tracks yourself. we wont merge since they wouldnt' share a UUID, only a close pose and maybe velocity which cant necessarily be used to assume one. 

kept in costmap until N updates without getting new info. So that if you have a track move from one sensor to another it doesnt disappar in the meantime.

# Detection methods supported

youcan use any... but we hae provided you with these:
- detectron2 + tracking: RGB detect correlate to depth to segment, size, pose, track. Works really well for RGB-D cameras because no extrinsics. Can also work with any RGB camera and any depth producing sensor like a 3D lidar
- 3D lidar only
- 2d lidar only

---


The projection I have some thoughts on, but nothing too concrete to share. First things first is all this boiler plate and software engineering before I get to the researchy-stuff

That is one way to include it - have a kinodynamic planner take into account this information, but that's very costly and frankly things like RRTs suck for mobile robotics

So instead, we mess with the environmental representation to project dynamic obstacles forward in time by their velocity and then plan in this projected space

Its a 'more basic' way of doing dynamic obstacle avoidance and easier to prototype. But you're right, after that's all done, my plan would be to offer a planner that takes in this information and does do kinodynamic planning

The downside of that is the kinodynamic planning is very expensive relative to things like A* or what most mobile robotics companies use. Rather than replanning at 20hz, its more like 1 hz, so typically you dont do that unless you need to. So starting with the costmap method is more flexible and usable for a larger number of users

so simplier to prototype, debug, and will overall get more use, even though its a little less technically satisfying.

For instance, doing full kinodynamic planning even without dynamic obstacles with OMPL in a small space can take in excess of 1 second

For many situations where you have a robot navigating in a very dynamic environment, that's unacceptably slow. But if you can use an A* planner at 30hz with a search space with temporal information, that can work reasonably well

its really analogous though,  if you think about it. The RRT is going to sample in the velocity and cost space, as it increments search its going to integrate those values to find the change in time to then project in time by that delta t the tracked information, then sample again in that understanding of the world. In this case, we just project some delta t 2 into the future and mark the costmap outright that full duration in time so that the search pattern won't let it 'cut off' a person

The major case where this degrades in performance is if you have a robot trying to navigate in something like a massive conference hall where you have hundreds of people walking in random directions. Projecting in time the full delta t at once will effectively make the entire floor lava :laughing:  where a kinodynamic is only checking the obstacles at the current time t. However, in basically every other situation with < 10 people in the immediate area, they're more or less the same on a systems diagram

So basically, long term both will be around, but in the immediate term, the costmap method is more practical for alot of people so I want to start there so that we can get people using it and provide immediate value

Its the same information from the detection / tracking side either way, its just how we're using it (or doing both). This is also a common way of dealing with this problem. While I feel like I was clever for coming up with it, I've seen several papers doing very similar things and even a few companies that have literally told me that's how they deal with multirobot collision avoidance

so its not just hand-waving, there's some backing behind this line of thinking

But basically this layer doesnt treat the costmap as a probability grid, it treats it as a risk grid

So in concept, what I'd really like ideally is a very accurate velocity and pose covariance matrix to use to project uncertainty over time, but in practice those are hard to compute accurately and alot of implementations just dont do them well at all

So instead, the projection model will need some gradient information proportional to whatever limited certainty information we do have

But "cost" doesnt have to mean probability of occupied, it could just be "cost of travel" which could be risk, and clearly cutting off a moving thing is more risky than going around it

https://ieeexplore.ieee.org/document/8967744
https://alyssapierson.files.wordpress.com/2018/05/pierson2018.pdf
