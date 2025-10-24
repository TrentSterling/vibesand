# VibeSand

A high-performance GPU-powered voxel physics simulator running on WebGPU. Simulates 2 million voxels updating at 60fps with real-time physics and dynamic lighting.

## Code Structure

### Architecture Overview

VibeSand is a **single-file application** (`index.html`) with no build system or external dependencies. The entire application is self-contained and runs directly in WebGPU-enabled browsers.

**Tech Stack:**
- **Pure WebGPU API** - No wrapper libraries or frameworks (no Three.js, Babylon.js, etc.)
- **WGSL Shaders** - All GPU compute and rendering shaders written in WebGPU Shading Language
- **Vanilla JavaScript** - DOM manipulation and application logic in plain JS
- **HTML5 Canvas** - Rendering target for the WebGPU context

### Core Components

The application is organized into several logical sections within `index.html`:

#### 1. **UI Layer** (HTML/CSS)
- Material design-inspired interface with controls panel
- Real-time FPS counter and performance metrics
- Tool selection and parameter sliders
- Camera mode toggle and instructions

#### 2. **Shader Pipeline** (WGSL)

The GPU pipeline consists of multiple compute and render shaders:

**Common Shader Code:**
- Voxel data packing/unpacking (type, variant, physics flags, RGB lighting in single `u32`)
- Coordinate system utilities
- Hash functions for procedural generation

**Compute Shaders:**
- `generationShaderCode` - World generation (terrain/flat modes)
- `simulationShaderCode` - Physics simulation (gravity, sliding, tumbling, fluid dynamics)
- `resetFlagsShaderCode` - Clears per-frame physics flags
- `lightingShaderCode` - Dynamic point light calculations
- `paintShaderCode` - Tool application (sand, stone, water, destroy)
- `clearLooseVoxelsShaderCode` - Removes non-solid voxels
- `targetingShaderCode` - GPU raycasting for cursor 3D position
- `managePlayerShaderCode` - Player voxel creation/maintenance (first-person mode)
- `findPlayerShaderCode` - Player voxel position tracking (GPU scan)

**Render Shader:**
- `renderShaderCode` - Single-pass DDA raymarching fragment shader
  - Renders voxels with lighting
  - **Enhanced procedural sky system** with:
    - Atmospheric gradient (horizon to zenith)
    - Animated sun disk with corona and glow effects
    - Volumetric procedural clouds using Fractal Brownian Motion
    - Dynamic time-of-day lighting (sunrise/sunset coloring)
  - Water surface shimmer effects
  - Tool preview sphere visualization

#### 3. **Camera System**

VibeSand implements **two distinct camera modes**:

##### **Orbital Camera Mode** (Default)
- **Architecture**: Traditional 3D viewport camera orbiting around a target point
- **Implementation**: Spherical coordinates (alpha, beta, radius) converted to Cartesian
- **Controls**:
  - Right-click + drag: Orbit camera around target
  - Middle-mouse / Space + Left-click + drag: Pan camera
  - Mouse wheel: Zoom in/out
  - Left-click: Use selected tool
- **Camera State**:
  ```javascript
  camera = {
    alpha: -0.5,        // Horizontal angle
    beta: 0.3,          // Vertical angle  
    radius: 230.4,      // Distance from target
    target: [64, 30, 64] // Look-at point
  }
  ```
- **Position Calculation**:
  ```javascript
  x = target[0] + radius * cos(beta) * sin(alpha)
  y = target[1] + radius * sin(beta)
  z = target[2] + radius * cos(beta) * cos(alpha)
  ```

##### **First Person Camera Mode**
- **Architecture**: FPS-style camera with WASD movement, mouse look, and physics-based player voxel
- **Implementation**: Bi-directional binding between camera and player voxel with collision detection
- **Controls**:
  - WASD: Move forward/left/backward/right (applies force to player voxel)
  - Space: Jump (upward movement)
  - Mouse: Look around (pointer lock API)
  - Left-click: Use tool at crosshair
  - **Auto-Jump**: Automatically jumps when encountering 1-block obstacles while moving forward
- **Camera State**:
  ```javascript
  firstPersonCamera = {
    position: [64, 35, 64], // 3D position (follows player voxel)
    yaw: -π/2,              // Horizontal rotation
    pitch: 0,                // Vertical rotation
    speed: 20.0              // Movement speed
  }
  playerVoxelPosition = [64, 35, 64];  // Actual player voxel position from GPU
  playerMoveIntent = [0, 0, 0];        // Movement intent from WASD input
  ```
- **Bi-Directional Binding System**:
  1. **Input → Player Voxel**: WASD keys calculate movement intent, applied to player voxel on GPU
  2. **Player Voxel → Camera**: Player position read back from GPU, camera follows with eye height offset
  3. **Physics Integration**: Player voxel affected by gravity, collision, and terrain
- **View Direction Calculation**:
  ```javascript
  forward = [
    cos(pitch) * cos(yaw),
    sin(pitch),
    cos(pitch) * sin(yaw)
  ]
  ```
- **Features**:
  - Crosshair HUD overlay
  - Pointer lock for seamless mouse look
  - **Physics-based FPS movement**:
    - Bright green player voxel represents player body
    - Camera position = player voxel position + 1.6 voxel units (eye height)
    - Player voxel affected by gravity (falls when no ground)
    - Collision detection: blocked by stone/sand, passes through air/water
    - Falls, slides diagonally down slopes, swaps with water
    - GPU-tracked with async CPU readback (1-frame latency)
    - Automatically respawns if destroyed or falls off world
    - True FPS game behavior: camera follows physics-simulated player
    - **Smart auto-jump**: Automatically jumps over 1-block obstacles when moving forward

**Camera Shared Systems:**
- Both modes calculate `currentRight`, `currentUp`, `currentForward` vectors
- Ray direction computation for both rendering and targeting
- Unified frustum and aspect ratio handling

#### 4. **Physics Engine** (GPU-based)

All physics runs on the GPU using compute shaders:

**Single-Buffer Architecture:**
- One storage buffer holds all 2M voxels
- Each voxel packed into single `u32` (4 bytes)
- "Updated" flag prevents double-processing per frame

**Physics Priority System:**
1. Direct vertical fall (gravity)
2. Diagonal fall/tumbling 
3. Horizontal tumbling over edges
4. Water horizontal spread (2-cell pressure)

**Particle Types:**
- Stone: Static solid, never moves
- Sand: Falls, slides diagonally, tumbles, swaps with water
- Water: Falls, flows, spreads horizontally with pressure
- **Player**: Physics-simulated voxel controlled by player input in first-person mode
  - Bright green color (RGB: 0.2, 0.8, 0.2) for high visibility
  - Spawned automatically in first-person mode
  - Responds to WASD movement intent with collision detection
  - Affected by gravity (falls, slides) like sand
  - Can move through air/water, blocked by stone/sand
  - GPU position tracked with async CPU readback (1-frame latency)
  - Camera position bound to player voxel position + eye height offset
  - Protected from deletion at y=0 (respawns if lost)
- Air: Empty space

#### 5. **Rendering Pipeline**

**Multi-Pass GPU Architecture:**
1. **Targeting Pass** - GPU raycast finds cursor 3D position
1.5. **Player Management Pass** - Apply player movement intent with collision detection (first-person mode only)
2. **Physics Passes** - Multiple simulation steps per frame (player voxel affected by gravity, slides, falls)
2.5. **Player Tracking Pass** - GPU scan to locate player voxel after physics (first-person mode only)
3. **Tool Application** - Modify voxels based on user input
4. **Lighting Pass** - Calculate light contributions from 32 dynamic point lights
5. **Render Pass** - Single-pass DDA raymarching renders the scene
6. **Position Readback** - Async GPU→CPU transfer of player voxel position (first-person mode only)

**DDA Raymarching:**
- Optimized voxel traversal algorithm
- Configurable max ray steps (64-384)
- Ray start bias prevents visual artifacts
- Integrated lighting, water shimmer, and tool preview in one shader

#### 6. **Player Voxel & Camera Control System** (详细说明 | Detailed Explanation)

**系统概述 | System Overview:**

VibeSand的第一人称模式实现了一个复杂的**物理驱动的玩家体素系统**，其中玩家输入和相机位置之间存在双向绑定。与传统FPS游戏中相机独立于物理世界不同，VibeSand的相机绑定到一个遵循与沙子和水相同物理规则的实体体素上。

VibeSand's first-person mode implements a complex **physics-driven player voxel system** with bidirectional binding between player input and camera position. Unlike traditional FPS games where the camera is independent of the physics world, VibeSand's camera is bound to a physical voxel that obeys the same physics rules as sand and water.

**玩家体素表示 | Player Voxel Representation:**
- **类型 | Type**: 特殊体素类型 `VOXEL_TYPE_PLAYER` (值: 4)
- **视觉 | Visual**: 亮绿色 (RGB: 0.2, 0.8, 0.2) 用于高可见性
- **物理 | Physics**: 受重力、碰撞检测和地形交互影响
- **行为 | Behavior**: 可以穿过空气/水，被固体体素(石头/沙子)阻挡

**双向绑定流程 | Bidirectional Binding Flow:**

```
┌─────────────────────────────────────────────────────────────┐
│        用户输入 (CPU) | User Input (CPU)                     │
│              WASD键 + 空格 + 鼠标                            │
│              WASD Keys + Space + Mouse                       │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│        计算移动意图 | Calculate Movement Intent              │
│        (JavaScript: updateFirstPersonCamera)                 │
│  • 水平移动: WASD 相对于相机方向 | Horizontal: WASD relative │
│  • 垂直移动: 空格跳跃(独立速度) | Vertical: Space jump      │
│  • 水平方向归一化防止对角加速 | Horizontal normalized       │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│       上传到GPU | Upload to GPU                              │
│       (computePassUniformsBuffer)                            │
│  playerMoveIntent → tool_info.camera_target_pos             │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│      GPU着色器: 管理玩家 | GPU: Manage Player Shader        │
│      (managePlayerShaderCode - 单线程执行)                   │
│  步骤1: 读取上一帧玩家位置 (playerPosBuffer)                │
│  步骤2: 检查该位置是否存在玩家体素                          │
│  步骤3: 如果存在 - 应用移动意图 + 碰撞检测                  │
│         • 计算目标位置: current_pos + move_intent           │
│         • 检查目标体素类型                                   │
│         • AIR/WATER: 原子交换移动玩家                        │
│         • STONE/SAND: 被阻挡，不移动                         │
│  步骤4: 如果不存在 - 在相机位置生成新玩家体素                │
│  Step 1: Read last player position (playerPosBuffer)        │
│  Step 2: Check if player voxel exists at that position      │
│  Step 3: If exists - Apply movement with collision          │
│         • Calculate target: current_pos + move_intent       │
│         • Check target voxel type                            │
│         • AIR/WATER: Atomic swap to move player             │
│         • STONE/SAND: Blocked, no movement                   │
│  Step 4: If not exists - Spawn new player at camera pos     │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│    GPU着色器: 物理模拟 | GPU: Physics Simulation             │
│    (simulationShaderCode - 每帧可能多次)                     │
│  • 玩家体素受重力影响(像沙子一样下落)                        │
│  • 沿斜坡对角滑动                                            │
│  • 落入水中时与水交换                                        │
│  • y=0处受保护不被删除(丢失时重生)                           │
│  • Player voxel affected by gravity (falls like sand)       │
│  • Slides diagonally down slopes                             │
│  • Swaps with water when falling into it                     │
│  • Protected from deletion at y=0 (respawns if lost)        │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│    GPU着色器: 查找玩家 | GPU: Find Player Shader            │
│    (findPlayerShaderCode - 并行扫描所有体素)                 │
│  • 每个工作组检查其负责的体素块                              │
│  • 找到 VOXEL_TYPE_PLAYER 时原子写入位置                     │
│  • 设置 found_flag = 1 表示找到                              │
│  • Parallel scan of all voxels for VOXEL_TYPE_PLAYER       │
│  • Atomically writes position when found                    │
│  • Sets found_flag = 1 if player exists                     │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│    GPU → CPU: 异步位置回读 | Async Position Readback        │
│  • copyBufferToBuffer: playerPosBuffer → playerPosReadBuffer│
│  • mapAsync(): 将位置数据读回JavaScript                     │
│  • 由于异步GPU-CPU传输存在1帧延迟                            │
│  • 1-frame latency due to async GPU-CPU transfer            │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│    更新相机位置 | Update Camera Position                     │
│    (JavaScript: updateFirstPersonCamera)                     │
│  firstPersonCamera.position =                                │
│      playerVoxelPosition + eyeHeightOffset                   │
│  • X, Z: 体素中心 (位置 + 0.5)                               │
│  • Y: 玩家位置 + PLAYER_EYE_HEIGHT (1.6单位)                 │
│  • 相机以眼睛高度偏移跟随玩家体素                            │
│  • X, Z: Voxel center (position + 0.5)                      │
│  • Y: Player pos + PLAYER_EYE_HEIGHT (1.6 units)            │
│  • Camera follows player voxel with eye-level offset        │
└─────────────────────────────────────────────────────────────┘
```

**关键技术细节 | Key Technical Details:**

**1. 移动意图计算 | Movement Intent Calculation**
```javascript
// 水平移动(归一化防止对角加速) | Horizontal movement (normalized)
horizontalMoveDir = normalize([forward/back, left/right based on yaw])
playerMoveIntent[0] = horizontalMoveDir[0] * speed * deltaTime  // X轴
playerMoveIntent[2] = horizontalMoveDir[1] * speed * deltaTime  // Z轴

// 垂直移动(独立跳跃速度) | Vertical movement (independent jump)
playerMoveIntent[1] = jumpVelocity * deltaTime  // 仅当按下空格时 | Only if Space pressed
```
**为什么分开处理? | Why separate?**
- 早期版本将跳跃和水平移动一起归一化，导致对角移动+跳跃时跳跃力度减弱70%
- Early version normalized jump with horizontal movement, causing 70% weaker jumps when moving diagonally
- 现在跳跃独立，确保无论移动方向如何都有一致的跳跃高度
- Now jump is independent, ensuring consistent jump height regardless of movement direction

**2. 碰撞检测 | Collision Detection** (GPU Shader)
```wgsl
// 计算目标位置 | Calculate target position
let current_pos_f = vec3<f32>(last_pos) + vec3<f32>(0.5);  // 体素中心
let new_pos_f = current_pos_f + move_intent;
let new_pos = vec3<u32>(clamp(floor(new_pos_f), 0, GRID_DIM - 1));

// 检查目标体素类型 | Check target voxel type
let target_type = getVoxelType(atomicLoad(&voxels[new_idx].data));

// 碰撞响应 | Collision response
if (target_type == VOXEL_TYPE_AIR || target_type == VOXEL_TYPE_WATER) {
    // 原子交换: 移动玩家，将原内容放在旧位置 | Atomic swap
    atomicStore(&voxels[new_idx].data, packVoxel(VOXEL_TYPE_PLAYER, ...));
    atomicStore(&voxels[old_idx].data, target_voxel_data);
} else {
    // 被固体阻挡，不移动 | Blocked by solid, no movement
}
```

**3. 自动跳跃系统 | Auto-Jump System** (GPU Shader)
```wgsl
// GPU检测前方障碍物 | GPU detects obstacles ahead
fn shouldAutoJump(player_pos, horizontal_move) -> bool:
    // 1. 计算前方位置 | Calculate position ahead
    forward_dir = normalize(horizontal_move)
    ahead_pos = player_pos + forward_dir
    
    // 2. 检查三个关键位置 | Check three key positions
    has_ground = (voxel_below_player is solid)
    has_obstacle = (voxel_ahead is solid)
    has_clearance = (voxel_above_obstacle is air/water)
    
    // 3. 自动跳跃条件 | Auto-jump conditions
    return has_ground && has_obstacle && has_clearance
```
**工作原理 | How it works:**
- 在玩家管理着色器中运行，每帧检测 | Runs in player management shader, checks every frame
- 只在有水平移动时激活 | Only activates when there's horizontal movement
- 检测前方1格是否有石头/沙子 | Detects if there's stone/sand 1 block ahead
- 确认玩家站在地面上 | Confirms player is standing on ground
- 确认障碍物上方有空间 | Confirms there's space above obstacle
- 自动设置跳跃速度 (0.15) | Automatically sets jump velocity (0.15)
- 不会覆盖手动跳跃 (Space键) | Doesn't override manual jump (Space key)

**4. 物理集成 | Physics Integration** (GPU)
- 玩家体素使用与沙子相同的物理规则 | Player voxels use same physics as sand
- 无支撑时向下坠落(重力) | Fall downward when unsupported (gravity)
- 落入水中时与水交换 | Swap with water when falling into it
- 沿斜坡对角滑动 | Slide diagonally down slopes
- y=0处受保护不被删除 | Protected from deletion at y=0

**5. 位置追踪 | Position Tracking** (GPU ↔ CPU)
```javascript
// GPU写入 playerPosBuffer (4 × u32 | 4 × unsigned 32-bit integers):
[x, y, z, found_flag]  // 所有操作都是原子的 | All atomic operations

// CPU异步读回 | CPU reads back asynchronously:
playerPosReadBuffer.mapAsync(GPUMapMode.READ)
    .then(() => {
        const data = new Uint32Array(playerPosReadBuffer.getMappedRange());
        playerVoxelPosition = [data[0], data[1], data[2]];
        playerVoxelFound = (data[3] === 1);
    });
    
// 1帧延迟在60 FPS下难以察觉 | 1-frame latency imperceptible at 60 FPS
```

**6. 生成/重生逻辑 | Spawn/Respawn Logic**
```wgsl
// 如果未找到玩家体素 | If player voxel not found:
if (last_found != 1u || getVoxelType(...) != VOXEL_TYPE_PLAYER) {
    // 在相机位置生成新玩家体素 | Spawn new player at camera position
    let spawn_pos = vec3<u32>(floor(camera_pos));
    atomicStore(&voxels[spawn_idx].data, 
                packVoxel(VOXEL_TYPE_PLAYER, 0u, 0u, vec3(0.0)));
    // 替换那里的任何体素以确保可见性 | Replaces whatever was there
}

// 物理模拟中的特殊保护 | Special protection in physics:
if (global_id.y == 0u) {
    if (current_type != VOXEL_TYPE_PLAYER) {
        atomicStore(&voxels[current_idx].data, 0u);  // 删除其他体素
    }
    // 玩家体素不被删除，即使在底部 | Player not deleted even at bottom
}
```

**性能特征 | Performance Characteristics:**

- **GPU效率 | GPU Efficiency**: 
  - 玩家管理: 1个工作组(单线程) | Player management: 1 workgroup (single-threaded)
  - 查找玩家: 并行扫描 (128³/4³ = 32,768工作组) | Find player: parallel scan (32,768 workgroups)
- **内存 | Memory**: 16字节玩家位置缓冲区(最小开销) | 16 bytes player buffer (minimal overhead)
- **延迟 | Latency**: 1帧延迟(在60 FPS下约16ms，难以察觉) | 1-frame delay (~16ms at 60 FPS, imperceptible)
- **线程安全 | Thread Safety**: 所有位置更新使用原子操作 | All position updates use atomic operations

**可配置参数 | Configurable Parameters:**

```javascript
// 玩家物理常量 | Player physics constants
const PLAYER_HEIGHT = 1.8;        // 玩家身体高度(体素单位)
const PLAYER_RADIUS = 0.4;        // 碰撞半径(当前未使用)
const PLAYER_EYE_HEIGHT = 1.6;    // 眼睛位置距离脚底

// 移动参数 | Movement parameters
firstPersonCamera.speed = 20.0;         // 水平移动速度 (体素/秒)
firstPersonCamera.jumpVelocity = 18.0;  // 跳跃力度 (体素/秒) - 增加以克服重力
```

**为什么采用这种设计? | Why This Design?**

这个复杂的系统实现了：| This complex system enables:
- **真实物理FPS | True physics-based FPS**: 玩家会下落、碰撞、与体素世界交互
- **涌现玩法 | Emergent gameplay**: 玩家可以在下落的沙子上冲浪、被水推动等
- **视觉反馈 | Visual feedback**: 玩家体素可见，确认位置
- **GPU加速 | GPU acceleration**: 所有物理和碰撞检测在GPU上
- **最小CPU开销 | Minimal CPU overhead**: CPU只处理键盘输入和异步位置回读

**常见问题 | Common Issues:**

1. **玩家无法移动 | Player Cannot Move**: 
   - 原因: `playerPosBuffer`未初始化导致GPU读取垃圾数据
   - 解决: 在创建缓冲区后立即用`writeBuffer`初始化为0
   - Cause: `playerPosBuffer` not initialized, GPU reads garbage data
   - Fix: Initialize to 0 with `writeBuffer` immediately after buffer creation

2. **跳跃太弱或无法跳跃 | Jump Too Weak or Cannot Jump**:
   - 原因1: 早期版本将跳跃向量与水平移动一起归一化
   - 解决1: 将垂直移动(跳跃)与水平移动分开计算
   - 原因2: 跳跃速度不足以克服重力
   - 解决2: 增加jumpVelocity从8.0到18.0
   - Cause 1: Early version normalized jump vector with horizontal movement
   - Fix 1: Calculate vertical (jump) and horizontal movement separately
   - Cause 2: Jump velocity insufficient to overcome gravity
   - Fix 2: Increase jumpVelocity from 8.0 to 18.0

3. **相机不跟随玩家 | Camera Not Following Player**:
   - 原因: `playerVoxelFound`为false或异步回读失败
   - 解决: 检查GPU缓冲区映射操作是否完成
   - Cause: `playerVoxelFound` is false or async readback failed
   - Fix: Ensure GPU buffer mapping operations complete

#### 7. **Memory Layout**

**Voxel Bit Packing** (32-bit unsigned integer):
```
Bits 0-2:   Voxel type (3 bits = 8 types)
Bits 3-5:   Color variant (3 bits = 8 variants)
Bit 6:      Physics updated flag
Bit 7:      (Reserved)
Bits 8-15:  Red light channel (8 bits)
Bits 16-23: Green light channel (8 bits)
Bits 24-31: Blue light channel (8 bits)
```

**GPU Buffers:**
- `voxelBuffer`: 8,388,608 bytes (128³ * 4 bytes)
- `lightsUniformBuffer`: Point light data (32 lights * 32 bytes)
- `renderPassUniformsBuffer`: Camera and sun data
- `computePassUniformsBuffer`: Tool and simulation params
- `targetingOutputBuffer`: Cursor raycast results
- `playerPosBuffer`: Player voxel position tracking (16 bytes: x, y, z, found_flag)

### Key Features

- **Bi-Directional Camera-Player Binding**: First-person camera bound to physics-simulated player voxel
  - Player input (WASD) applies movement intent to player voxel on GPU
  - Player voxel responds to physics (gravity, collision, terrain)
  - Camera position follows player voxel with eye height offset
  - Async GPU→CPU position readback with 1-frame latency
  - True FPS game physics: falls, jumps, collides with terrain
- **GPU-First Architecture**: Minimal CPU involvement, all simulation and rendering on GPU
- **Atomic Operations**: Lock-free parallel physics using atomics
- **Player Voxel Physics**: GPU-managed player with collision detection and gravity
- **Dynamic Lighting**: 32 falling colored point lights ("light rain")
- **Interactive Tools**: Brush-based painting system with preview
- **Resolution Scaling**: Adjustable render resolution for performance
- **Continuous Simulation**: User-configurable physics step count
- **Enhanced Sky System**: Procedural skybox with sun and volumetric clouds (see below)

### Sky Rendering System

The sky system uses **procedural generation** for maximum performance and visual quality:

**Technical Approach:**
- **Skybox-based rendering** (not voxel-based) for optimal performance
- Computed per-pixel in fragment shader during ray marching
- No additional textures or GPU memory required
- Fully dynamic and synchronized with sun position

**Sky Components:**

1. **Atmospheric Gradient**
   - Smooth transition from horizon (light blue) to zenith (deep blue)
   - Uses power function for realistic atmospheric scattering appearance
   - Sunset/sunrise coloring near horizon based on sun elevation

2. **Sun Rendering**
   - **Sun disk**: Bright core using high-power falloff (pow 128)
   - **Sun glow**: Medium falloff halo (pow 8) 
   - **Sun corona**: Wide soft glow (pow 2)
   - Warm yellow-white color (1.0, 0.95, 0.8)
   - Automatically positioned using existing sun direction vector

3. **Procedural Clouds**
   - **4-octave Fractal Brownian Motion** for realistic cloud shapes
   - **3D value noise** with smooth interpolation
   - **Animated**: Clouds drift slowly over time
   - **Sun-lit**: Cloud brightness varies based on sun position
   - **Performance optimization**: Only rendered in upper hemisphere (y > 0.05)
   - **View-dependent fading**: Clouds fade when looking straight up or down
   - Uses domain warping for natural, billowy appearance

**Performance Characteristics:**
- Minimal performance impact (~1-2ms per frame on modern GPUs)
- Clouds use early exit for rays pointing down
- Noise functions use hardware-accelerated math operations
- No texture fetches or additional memory bandwidth

## Running the Project

1. Open `index.html` in a WebGPU-compatible browser (Chrome 113+, Edge 113+)
2. Ensure your GPU drivers are up to date
3. No build step or package installation required

## Browser Compatibility

- **Requires WebGPU support** (Chrome/Edge 113+, Safari 17.4+)
- Will display error message if WebGPU is unavailable
- Check compatibility at [WebGPUReport.org](https://webgpureport.org)

## Performance

- 2,097,152 voxels (128³ grid)
- Target: 60 FPS on modern GPUs
- Adjustable quality settings:
  - Render scale (25%-100%)
  - Max ray steps (64-384)
  - Simulation steps per frame (0-25)
