# CARLA Autonomous Vehicle Project

## Project Milestones Tracking

### Subtask Progress Table

| Subtask ID | Description | Assigned To | Expected Completion | Status | Notes |
|------------|-------------|--------|---------------------|--------|-------|
| **Milestone 1** | |        |                     | | |
| M1.1 | CARLA 0.9.13 Installation | ALL    | 11/05/2025          | ✅ Completed | Python 3.7 verified |
| M1.2 | Virtual Environment Setup | All    | 13/05/2025          | ✅ Completed | Need test scripts |
| M1.3 | API Familiarization | All    | 16/05/2025          | ✅ Completed | manual_control.py issues |
| M1.4 | Basic Movement Implementation | [Rifat, Sara] | 17/05/2025          | ✅ Completed | Speed control working |
| M1.5 | Steering Logic | [Rifat] | 17/05/2025          | ✅ Completed | Excercise route finished |
| **Milestone 2** | |        |                     | | |
| M2.1 | LIDAR Detection Setup | [Victor] | MM/DD/YYYY          | ❌ Not Started | |
| M2.2 | HEALING State Logic | [Victor] | MM/DD/YYYY          | ❌ Not Started | |
| M2.3 | Avoidance Maneuvers | [Victor] | MM/DD/YYYY          | ❌ Not Started | |
| **Milestone 3** | |        |                     | | |
| M3.1 | Waypoint Routing | [Rifat] | MM/DD/YYYY          | ❌ Not Started | |
| M3.2 | Traffic Light Integration | [Victor] | MM/DD/YYYY          | ❌ Not Started | |

### Status Key:
- ✅ Completed
- ⏳ In Progress
- 🟡 Due Soon (within 3 days)
- 🔴 Failed/Blocked
- ❌ Not Started

## Detailed Milestone Breakdown

### Milestone 1: Basic Vehicle Control
```markdown
1. [✅] CARLA Installation
   - [✅] Verify Python 3.7 compatibility
   - [✅] Test API connection
2. [🟡] Movement System
   - [ ] Forward motion control
   - [⏳] Steering calculation (need cross product fix)
3. [❌] Rotation Handling
```

### Milestone 1: Basic Vehicle Control
```markdown
1. [❌] Detection System
   - [❌] LIDAR processing
   - [❌] Collision prediction
2. [❌] Avoidance Logic
```

### Milestone 3: Route Planning
```markdown
1. [❌] Navigation
   - [❌] Waypoint following
   - [❌] Lane change logic
2. [❌] Traffic Systems
```

### Test Commands Cheatsheet
```bash
# Run all tests
python run_tests.py --all

# Milestone-specific tests
python ai_test.py -m 1  # Basic movement
python ai_test.py -m 3  # Obstacle avoidance
python ai_test.py -m 2  # Full route
```
### Blockers
1. Steering angle calculation needs math verification
2. LIDAR sensor gives noisy data at high speeds
3. Waypoint connections sometimes break at intersections