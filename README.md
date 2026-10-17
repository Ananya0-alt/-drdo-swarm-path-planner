# 🛰️ DRDO Drone Swarm Path Planner

🚀 **Live App:** 🚀 Live App: https://yqym6kgdxjk8pe3qqnt8al.streamlit.app
📁 **For DRDO Internship - UAV Swarm Intelligence**

### Overview
3 drones navigate a hostile map to reach targets without collision. Built to replicate DRDO's swarm drone mission planning system.

**Algorithms Used:**
- A* Search: Shortest path with Manhattan heuristic (8-direction movement)
- Collision Avoidance: Prioritized Planning + Space-Time Reservation Table
- DroneKit: Would be used for real Pixhawk via MAVLink

### How Collision is Avoided (DRDO Interview Point)
1. Plan Drone 1 path, reserve its cells in time as (x,y,t)
2. Drone 2 plans path avoiding reserved cells, waits if needed
3. Repeat for Drone N -> Results in 0% collision
4. This is called Cooperative A*

### Features
- Hostile No-Fly Zones (black cells) - random 20% density
- 2-5 drones configurable
- Real-time path visualization with matplotlib
- Stats: Total Distance, Avg Path, Safety %
- Simulated DroneKit SITL logs

### Results
- Successfully plans 3 collision-free paths in 30x30 grid
- 100% Safety, Avg path ~45 steps

### Tech Stack
Python, A*, NumPy, Matplotlib, Streamlit, DroneKit-Python (Sim)

### Built By
Ananya - BTech - For DRDO Jodhpur / ADE Bangalore Application 2026

### How to Run
