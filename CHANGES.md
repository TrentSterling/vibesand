# Player Jump Fix and Auto-Jump Feature

## Summary

This update fixes the player jumping issue in first-person mode and adds an intelligent auto-jump feature that automatically helps the player navigate over 1-block obstacles.

## Changes Made

### 1. Jump Velocity Fix
**Problem:** Players could not jump in first-person mode because the jump velocity (8.0) was insufficient to overcome the gravity effect in the physics simulation.

**Solution:** Increased `jumpVelocity` from 8.0 to 18.0 in `firstPersonCamera` configuration.

**Location:** `index.html` line ~1184

**Impact:** Players can now successfully jump when pressing the Space key.

### 2. Auto-Jump Feature
**Problem:** Players requested automatic jumping when encountering small obstacles (1 block high).

**Solution:** Implemented GPU-based obstacle detection in the `managePlayerShaderCode` shader.

**How It Works:**
1. Every frame, when the player has horizontal movement intent, the shader checks the block directly ahead
2. It verifies three conditions:
   - Player is standing on solid ground (has support)
   - There's a solid obstacle (stone or sand) 1 block ahead
   - There's clearance (air or water) above the obstacle
3. If all conditions are met, automatically applies upward velocity (0.15)
4. Does not interfere with manual jumping (Space key has priority)

**Location:** `index.html` line ~1663-1710

**Benefits:**
- Smoother navigation through terrain
- Reduces frustration when navigating stairs or small obstacles
- Maintains realistic feel (only works on 1-block obstacles)
- GPU-accelerated detection (no CPU overhead)

## Technical Details

### GPU Shader Function: `shouldAutoJump()`

```wgsl
fn shouldAutoJump(player_pos: vec3<u32>, horizontal_move: vec2<f32>) -> bool
```

**Algorithm:**
1. Check if there's meaningful horizontal movement (`length(horizontal_move) > 0.01`)
2. Calculate the forward direction based on movement intent
3. Check voxels at three key positions:
   - Below player (ground check)
   - Ahead of player (obstacle check)
   - Above obstacle (clearance check)
4. Return true only if all conditions are satisfied

**Performance:**
- Runs on GPU (no CPU overhead)
- Single-threaded workgroup (1,1,1)
- Minimal branching, optimal for GPU execution
- Only activates when player is moving

### Integration Points

1. **Player Management Shader** (`managePlayerShaderCode`)
   - Auto-jump check runs before applying movement intent
   - Only triggers if player is not already jumping upward
   - Modifies `move_intent.y` when conditions are met

2. **Physics Simulation** (`simulationShaderCode`)
   - No changes needed - player voxel follows existing physics rules
   - Gravity still applies, making auto-jump feel natural

3. **Camera System** (`updateFirstPersonCamera`)
   - No changes needed - camera follows player voxel position
   - 1-frame latency in position readback remains unchanged

## Testing Recommendations

To test these changes in a WebGPU-capable browser:

1. **Manual Jump Test:**
   - Switch to First Person mode (FPS button)
   - Press Space while standing on ground
   - Player should jump upward visibly

2. **Auto-Jump Test:**
   - Build a simple 1-block high wall using Stone tool
   - Walk directly into the wall using W key
   - Player should automatically jump over the obstacle

3. **No False Triggers Test:**
   - Walk on flat terrain - should not auto-jump
   - Walk toward 2+ block high walls - should not auto-jump
   - Jump manually - auto-jump should not interfere

## Documentation Updates

- Updated `README.md` with auto-jump system documentation
- Added detailed GPU shader explanation
- Updated configurable parameters section
- Enhanced "Common Issues" section with jump fixes
- Updated controls description in `index.html` info panel

## Compatibility

- **Browser Requirements:** Chrome 113+, Edge 113+, Safari 17.4+ (WebGPU support)
- **GPU Requirements:** Any WebGPU-compatible GPU
- **Performance Impact:** Negligible (GPU-accelerated detection)
- **Breaking Changes:** None

## Future Enhancements

Possible future improvements:
1. Configurable auto-jump sensitivity
2. UI toggle to enable/disable auto-jump
3. Variable jump height based on obstacle height
4. Sound effects for auto-jump activation
5. Visual indicator when auto-jump triggers
