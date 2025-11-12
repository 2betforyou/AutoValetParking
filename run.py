# run.py
import os
from datetime import datetime
from pathlib import Path
import simpy
from viz_autovp import make_world, arrival, render_combined_gif
from props import SIM_TIME, DT

def main():
    env = simpy.Environment()
    floors, tower = make_world(env)
    logs = []
    print("=== Parking Simulation Start ===")
    env.process(arrival(env, tower, logs))
    env.run(until=SIM_TIME)
    print("=== Simulation End ===")
    
    out_dir = Path("output")
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"combined_twofloors_{datetime.now():%Y%m%d_%H%M%S}.gif"

    render_combined_gif(floors, tower.traj, tower.occ, env.now, DT, str(out_path))

if __name__ == "__main__":
    main()