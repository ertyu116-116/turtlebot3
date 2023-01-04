참고
1)https://github.com/ros-planning/navigation2/tree/main/nav2_waypoint_follower
  -ros2를 써서 그냥 sdk를 가져다 쓰기는 힘들다.
2)https://github.com/gpldecha/turtlebot-navigation



자율주행(closed loop system)

노드 초기화 x,y,z:(0,0,0)

위치값들을 기억한다 ex.(1.5,0.5,0) 
그럴려면 fixed frame = map or odom 이어야 한다.
base link 로 하게되는 순간 정확한 좌표를 얻기 어렵다.

중간에 쓰러진 사람이 있다면 경고를 준다.

기억한 좌표에 도착하면 다음 노드로 가고 그다음 노드로 가다가
마지막에 원래 위치 (0,0,0)으로 돌아온다.



fixed frame:map

initial point:
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped -- '[0, [0, 0], "map"]' '[[-1.5,1.5, 0], [0, 0, 0, 1]]'

rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped -- '[0, [0, 0], "map"]' '[[0,0, 0], [0, 0, 0, 3]]'

if(amcl이 목적지 +- 0.3정도 일때?)
	publish해라

rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped -- '[0, [0, 0], "map"]' '[[-0.5532969469964597,0.37531878237440264,0],[0.0, 0.0,-0.014366698469828125, 0.9998967936617644]]'
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped -- '[0, [0, 0], "map"]' '[[1.5,-1.5, 0], [0, 0, 0, 1]]'

rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped -- '[0, [0, 0], "map"]' '[[-1.5,-1.5, 0], [0, 0, 0, 1]]'

rosservice call /move_base/clear_costmaps "{}"
