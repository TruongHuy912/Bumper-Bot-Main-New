# Yêu cầu cho Codex: thêm Active SLAM phiên bản 1 cho Bumper-Bot ROS2

## 0. Mục tiêu

Hãy chỉnh sửa codebase `AntoBrandi/Bumper-Bot` để robot Bumper-Bot có thể **tự khám phá môi trường đang được SLAM vẽ online** theo phiên bản 1:

```text
Bumper-Bot + SLAM Toolbox + Nav2 + node Active SLAM tự viết
```

Phiên bản 1 chỉ cần triển khai:

```text
/map + TF pose robot
    -> frontier detection trên OccupancyGrid
    -> path entropy theo đường robot -> frontier
    -> distance penalty
    -> chọn frontier tốt nhất
    -> gửi goal cho Nav2 NavigateToPose
    -> lặp lại khi map cập nhật
```

Không cần port nguyên repo ROS1 `MF-Ahmed/aslam_rosbot`. Không cần làm D-optimality/pose graph uncertainty trong phiên bản 1. Không cần đổi SLAM backend hiện tại của Bumper-Bot.

---

## 1. Bối cảnh codebase hiện tại

Bumper-Bot là repo ROS2. Repo hiện đã có các package chính:

- `bumperbot_bringup`
- `bumperbot_description`
- `bumperbot_controller`
- `bumperbot_mapping`
- `bumperbot_navigation`
- `bumperbot_localization`
- `bumperbot_planning`

Trong launch mô phỏng hiện tại:

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py use_slam:=true
```

khi `use_slam:=true`, hệ thống cần chạy:

- Gazebo/robot description
- controller
- SLAM Toolbox từ `bumperbot_mapping`
- Nav2 từ `bumperbot_navigation`
- RViz

Trong `bumperbot_mapping/config/slam_toolbox.yaml`, các frame/topic cần ưu tiên bám theo:

```yaml
odom_frame: odom
map_frame: map
base_frame: base_footprint
scan_topic: /scan
mode: mapping
resolution: 0.05
```

Vì vậy node Active SLAM mới mặc định nên dùng:

```yaml
map_topic: /map
global_frame: map
robot_base_frame: base_footprint
nav2_action_name: /navigate_to_pose
use_sim_time: true
```

Nếu trong máy thật frame khác, cho phép đổi bằng parameter `robot_base_frame` sang `base_link`.

---

## 2. Yêu cầu kiến trúc

Tạo package ROS2 mới tên:

```text
bumperbot_active_slam
```

Nên dùng Python/rclpy để dễ chỉnh:

```text
bumperbot_active_slam/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/bumperbot_active_slam
├── bumperbot_active_slam/
│   ├── __init__.py
│   └── active_slam_node.py
├── config/
│   └── active_slam.yaml
├── launch/
│   └── active_slam.launch.py
└── README.md
```

Có thể thêm launch tích hợp trong `bumperbot_bringup`, ví dụ:

```text
bumperbot_bringup/launch/active_slam_simulated_robot.launch.py
```

Launch tích hợp này nên include:

```text
simulated_robot.launch.py use_slam:=true
active_slam.launch.py use_sim_time:=true
```

Nếu không muốn sửa bringup nhiều, ít nhất phải chạy được bằng 2 terminal:

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py use_slam:=true
ros2 launch bumperbot_active_slam active_slam.launch.py use_sim_time:=true
```

---

## 3. Node chính: `active_slam_node.py`

Node tên:

```text
active_slam_node
```

### 3.1. Subscribe

Subscribe OccupancyGrid:

```text
/map : nav_msgs/msg/OccupancyGrid
```

Dùng QoS phù hợp cho map:

```python
QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
```

### 3.2. TF

Dùng `tf2_ros.Buffer` và `tf2_ros.TransformListener` để lấy pose robot:

```text
map -> base_footprint
```

Tên frame phải lấy từ parameter:

```yaml
global_frame: map
robot_base_frame: base_footprint
```

Nếu TF chưa có, node không crash; chỉ log warning và đợi.

### 3.3. Action client Nav2

Dùng action client:

```text
nav2_msgs/action/NavigateToPose
```

Action server mặc định:

```text
/navigate_to_pose
```

Node phải:

- đợi action server sẵn sàng;
- gửi goal dạng `geometry_msgs/PoseStamped` trong frame `map`;
- theo dõi trạng thái goal;
- nếu goal thành công thì chọn goal mới sau `goal_cooldown_sec`;
- nếu goal thất bại hoặc bị hủy thì blacklist goal đó trong `blacklist_duration_sec`.

### 3.4. Publish để debug trong RViz

Publish:

```text
/active_slam/frontiers       visualization_msgs/msg/MarkerArray
/active_slam/selected_goal   geometry_msgs/msg/PoseStamped
/active_slam/status          std_msgs/msg/String
```

Marker yêu cầu:

- frontier clusters: điểm/cube nhỏ;
- selected goal: sphere hoặc arrow;
- không cần màu cố định phức tạp, nhưng phải dễ nhìn trong RViz.

---

## 4. Thuật toán phiên bản 1

### 4.1. Quy ước OccupancyGrid

Trong ROS OccupancyGrid:

```text
-1  = unknown
0   = free
100 = occupied
```

Dùng parameter:

```yaml
unknown_value: -1
free_max_value: 25
occupied_min_value: 65
```

Một cell được xem là free nếu:

```python
0 <= value <= free_max_value
```

Một cell được xem là occupied nếu:

```python
value >= occupied_min_value
```

Một cell unknown nếu:

```python
value == unknown_value
```

### 4.2. Phát hiện frontier

Một cell là frontier nếu:

```text
cell hiện tại là free
và ít nhất một cell lân cận 8 hướng là unknown
```

Các bước:

1. Convert `OccupancyGrid.data` thành numpy array 2D `(height, width)`.
2. Tìm tất cả frontier cells.
3. Gom cụm frontier bằng BFS/DFS 8-connected.
4. Lọc cụm nhỏ:

```yaml
min_frontier_cells: 8
```

5. Với mỗi cụm, tính centroid theo tọa độ map.
6. Goal không được đặt vào unknown/occupied. Hãy chọn cell free đại diện tốt nhất trong cụm hoặc gần centroid, nhưng phải đảm bảo nằm trong vùng free và gần unknown.

### 4.3. Safety check cho goal

Goal candidate phải thỏa:

- nằm trong map bounds;
- cell goal là free;
- quanh goal trong bán kính `goal_clearance_radius_m` không có occupied cell;
- khoảng cách từ robot đến goal nằm trong khoảng:

```yaml
min_goal_distance_m: 0.4
max_goal_distance_m: 8.0
```

Nếu goal quá sát unknown hoặc Nav2 hay fail, hãy kéo goal lùi về phía robot một đoạn nhỏ bằng parameter:

```yaml
goal_backoff_m: 0.20
```

Cách backoff:

```text
goal_backed = frontier_goal - goal_backoff_m * unit_vector(frontier_goal - robot_pose)
```

Sau đó snap về cell free gần nhất.

### 4.4. Tính path entropy

Dùng Bresenham line từ cell robot đến cell goal.

Với mỗi cell trên đường:

- unknown: entropy cao;
- free/occupied đã biết: entropy thấp hơn;
- nếu gặp occupied trên đường thì candidate bị invalid hoặc utility rất thấp.

Công thức entropy nhị phân:

```python
H(p) = -p * log(p) - (1 - p) * log(1 - p)
```

Để tránh log(0), clip:

```python
p = min(max(p, 1e-3), 1 - 1e-3)
```

Map value sang xác suất:

```python
if value == -1:
    p = 0.5
else:
    p = value / 100.0
```

Chuẩn hóa entropy theo entropy lớn nhất tại `p=0.5`:

```python
H_norm = H(p) / H(0.5)
```

Path entropy:

```python
path_entropy = mean(H_norm trên các cell của đường robot -> goal)
```

Nếu muốn tăng độ ổn định, có thể lấy thêm neighborhood nhỏ quanh line bằng parameter:

```yaml
entropy_neighborhood_radius_cells: 1
```

nhưng phiên bản đầu chỉ cần Bresenham line là được.

### 4.5. Distance penalty

Tính khoảng cách Euclidean từ robot đến goal:

```python
distance_m = hypot(goal_x - robot_x, goal_y - robot_y)
```

Distance score:

```python
distance_score = exp(-lambda_distance * distance_m)
```

Parameter:

```yaml
lambda_distance: 0.25
```

### 4.6. Utility

Phiên bản 1 dùng utility đơn giản:

```python
utility = path_entropy * distance_score
```

Có thể bổ sung frontier size nếu muốn robot ưu tiên cụm frontier lớn hơn:

```python
frontier_size_score = min(cluster_size / frontier_size_norm_cells, 1.0)
utility = entropy_weight * path_entropy * distance_score + frontier_size_weight * frontier_size_score
```

Parameter mặc định:

```yaml
entropy_weight: 1.0
frontier_size_weight: 0.2
frontier_size_norm_cells: 80
```

Không thêm D-optimality trong phiên bản này.

### 4.7. Chọn goal

Chọn candidate có utility lớn nhất.

Không gửi lại goal mới nếu:

- đang có goal Nav2 active;
- chưa hết `goal_cooldown_sec`;
- selected goal quá gần goal vừa gửi trước đó:

```yaml
same_goal_tolerance_m: 0.3
```

Nếu không còn frontier hợp lệ:

- publish status `NO_FRONTIER_FOUND`;
- không crash;
- dừng gửi goal.

---

## 5. Parameter file đề xuất

Tạo `config/active_slam.yaml`:

```yaml
active_slam_node:
  ros__parameters:
    use_sim_time: true

    map_topic: /map
    global_frame: map
    robot_base_frame: base_footprint
    nav2_action_name: /navigate_to_pose

    control_period_sec: 2.0
    goal_cooldown_sec: 3.0
    blacklist_duration_sec: 20.0

    unknown_value: -1
    free_max_value: 25
    occupied_min_value: 65

    min_frontier_cells: 8
    min_goal_distance_m: 0.4
    max_goal_distance_m: 8.0
    goal_clearance_radius_m: 0.18
    goal_backoff_m: 0.20
    same_goal_tolerance_m: 0.30

    lambda_distance: 0.25
    entropy_weight: 1.0
    frontier_size_weight: 0.2
    frontier_size_norm_cells: 80.0
    entropy_neighborhood_radius_cells: 0

    publish_markers: true
    marker_topic: /active_slam/frontiers
    selected_goal_topic: /active_slam/selected_goal
    status_topic: /active_slam/status
```

---

## 6. Launch file đề xuất

Tạo `launch/active_slam.launch.py` với các argument:

```text
use_sim_time:=true
params_file:=<path to active_slam.yaml>
```

Launch node:

```text
package: bumperbot_active_slam
executable: active_slam_node
name: active_slam_node
output: screen
parameters: [params_file, {use_sim_time: use_sim_time}]
```

---

## 7. Tích hợp với Bumper-Bot

### Cách chạy tối thiểu

Sau khi build:

```bash
source install/setup.bash
ros2 launch bumperbot_bringup simulated_robot.launch.py use_slam:=true
```

Terminal khác:

```bash
source install/setup.bash
ros2 launch bumperbot_active_slam active_slam.launch.py use_sim_time:=true
```

### Cách chạy tích hợp tùy chọn

Nếu tạo launch tích hợp trong `bumperbot_bringup`:

```bash
ros2 launch bumperbot_bringup active_slam_simulated_robot.launch.py
```

Launch này phải include `simulated_robot.launch.py` với:

```text
use_slam:=true
```

rồi include `bumperbot_active_slam/launch/active_slam.launch.py`.

---

## 8. Lệnh kiểm tra bắt buộc

Sau khi chạy simulation:

```bash
ros2 topic list | grep map
ros2 topic echo /map --once
```

Kiểm tra TF:

```bash
ros2 run tf2_ros tf2_echo map base_footprint
```

Nếu không có `base_footprint`, thử:

```bash
ros2 run tf2_ros tf2_echo map base_link
```

và đổi parameter `robot_base_frame`.

Kiểm tra Nav2 action:

```bash
ros2 action list | grep navigate_to_pose
```

Kiểm tra node Active SLAM:

```bash
ros2 topic echo /active_slam/status
ros2 topic echo /active_slam/selected_goal
```

Kiểm tra marker trong RViz:

```text
Add -> MarkerArray -> /active_slam/frontiers
Add -> PoseStamped -> /active_slam/selected_goal
```

---

## 9. Acceptance criteria

Code được xem là đạt nếu:

1. `colcon build` thành công.
2. `ros2 launch bumperbot_active_slam active_slam.launch.py use_sim_time:=true` chạy không lỗi import.
3. Node nhận được `/map` từ SLAM Toolbox.
4. Node lấy được pose robot từ TF `map -> base_footprint` hoặc cấu hình frame tương ứng.
5. Node publish được frontier markers trong RViz.
6. Node chọn được goal frontier và publish `/active_slam/selected_goal`.
7. Node gửi goal được cho Nav2 action `/navigate_to_pose`.
8. Bumper-Bot di chuyển đến frontier, SLAM Toolbox tiếp tục cập nhật map, sau đó node tiếp tục chọn goal mới.
9. Khi không có map, không có TF hoặc Nav2 chưa sẵn sàng, node không crash mà log warning và đợi.
10. Khi goal fail, node blacklist goal đó tạm thời và thử frontier khác.
11. Không hard-code absolute path.
12. Không thay đổi mạnh cấu trúc cũ của Bumper-Bot nếu không cần thiết.

---

## 10. Những lỗi dễ gặp và cách xử lý

### Lỗi 1: Nav2 không plan được tới frontier

Nguyên nhân thường là goal nằm quá sát vùng unknown hoặc obstacle inflation.

Cách xử lý:

- tăng `goal_backoff_m` từ `0.20` lên `0.30`;
- tăng `goal_clearance_radius_m`;
- snap goal về cell free gần robot hơn;
- không đặt goal vào unknown.

### Lỗi 2: Node không nhận map

Kiểm tra:

```bash
ros2 topic echo /map --once
```

Nếu map có nhưng node không nhận, sửa QoS subscriber sang `TRANSIENT_LOCAL + RELIABLE`.

### Lỗi 3: TF không có `map -> base_footprint`

Kiểm tra:

```bash
ros2 run tf2_ros tf2_echo map base_footprint
ros2 run tf2_ros tf2_echo map base_link
```

Sau đó đổi parameter `robot_base_frame`.

### Lỗi 4: Robot cứ gửi lại cùng một goal

Thêm:

- `same_goal_tolerance_m`;
- lưu last goal;
- nếu goal mới quá gần goal cũ thì bỏ qua;
- cooldown sau mỗi goal.

### Lỗi 5: Frontier quá nhiễu

Tăng:

```yaml
min_frontier_cells: 15
```

hoặc lọc cluster theo diện tích mét:

```python
cluster_size_m = cluster_cell_count * resolution
```

---

## 11. Không làm trong phiên bản 1

Không làm các phần sau trong pull request đầu tiên:

- Không port ROS1 `aslam_rosbot` sang ROS2.
- Không thay `slam_toolbox` bằng Open Karto/g2o.
- Không triển khai D-optimality hoặc pose graph uncertainty.
- Không viết lại Nav2.
- Không sửa sâu controller của Bumper-Bot.
- Không phụ thuộc vào map đã lưu sẵn/AMCL. Phải chạy với `use_slam:=true`.

---

## 12. README cần có

Tạo `bumperbot_active_slam/README.md` ghi rõ:

- package này là Active SLAM phiên bản 1;
- lấy cảm hứng từ ý tưởng path entropy của `aslam_rosbot` nhưng viết mới cho ROS2/Bumper-Bot;
- node dùng `/map` online từ SLAM Toolbox;
- node gửi goal tới Nav2;
- cách chạy simulation;
- cách xem marker trong RViz;
- danh sách parameter quan trọng.

---

## 13. Gợi ý pseudo-code

```python
on_timer():
    if no map:
        publish_status("WAITING_FOR_MAP")
        return

    robot_pose = lookup_tf(global_frame, robot_base_frame)
    if no robot_pose:
        publish_status("WAITING_FOR_TF")
        return

    if nav_goal_active:
        publish_status("NAVIGATING")
        return

    grid = occupancy_grid_to_numpy(map_msg)
    frontier_cells = detect_frontier_cells(grid)
    clusters = cluster_frontiers(frontier_cells)
    candidates = []

    for cluster in clusters:
        if len(cluster) < min_frontier_cells:
            continue

        goal = compute_goal_from_cluster(cluster, robot_pose)
        goal = backoff_and_snap_to_free(goal, robot_pose, grid)

        if not is_goal_safe(goal, grid):
            continue

        if is_blacklisted(goal):
            continue

        path_cells = bresenham(robot_cell, goal_cell)
        path_entropy = compute_path_entropy(path_cells, grid)
        distance_score = exp(-lambda_distance * distance_m(robot_pose, goal))
        size_score = min(len(cluster) / frontier_size_norm_cells, 1.0)

        utility = entropy_weight * path_entropy * distance_score + frontier_size_weight * size_score
        candidates.append((utility, goal, cluster))

    publish_frontier_markers(clusters, candidates)

    if not candidates:
        publish_status("NO_VALID_FRONTIER")
        return

    best = max(candidates, key=lambda x: x[0])
    send_nav2_goal(best.goal)
    publish_selected_goal(best.goal)
    publish_status("GOAL_SENT")
```

---

## 14. Gợi ý code math/helper

### Map index conversion

```python
def world_to_map(x, y, info):
    mx = int((x - info.origin.position.x) / info.resolution)
    my = int((y - info.origin.position.y) / info.resolution)
    return mx, my


def map_to_world(mx, my, info):
    x = info.origin.position.x + (mx + 0.5) * info.resolution
    y = info.origin.position.y + (my + 0.5) * info.resolution
    return x, y
```

### Bresenham

```python
def bresenham(x0, y0, x1, y1):
    cells = []
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    x, y = x0, y0
    while True:
        cells.append((x, y))
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x += sx
        if e2 <= dx:
            err += dx
            y += sy
    return cells
```

### Entropy

```python
def cell_entropy(value):
    if value < 0:
        p = 0.5
    else:
        p = float(value) / 100.0
    p = min(max(p, 1e-3), 1.0 - 1e-3)
    h = -p * math.log(p) - (1.0 - p) * math.log(1.0 - p)
    return h / math.log(2.0)
```

