import asyncio
import threading
import time
import sys
import os

# Add project root and vex-aim-tools to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'vex-aim-tools')))

import aim_fsm
from aim_fsm import evbase
from aim_fsm import program
from aim_fsm.robot import Robot as AIMRobot

try:
    from termProject.Robot1 import Robot1Program
    from termProject.Robot2 import Robot2Program
    from termProject.shared_context import ScoreKeeper
except ImportError as e:
    print("Could not import generated .py FSM files:", e)
    sys.exit(1)

def main():
    # Setup asyncio loop for the robots WebSocket threads
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    
    def loopthread():
        loop.run_forever()
    
    th = threading.Thread(target=loopthread, daemon=True)
    th.start()
    
    print("Connecting to Robot 1...")
    ip1 = os.getenv("ROBOT", "vex-aim-8.engr.oregonstate.edu")
    try:
        r1 = AIMRobot(loop=loop, host=ip1)
    except Exception as e:
        print("Failed to initialize Robot 1:", e)
        return
        
    print("Connecting to Robot 2...")
    ip2 = os.getenv("ROBOT2", "vex-aim-18.engr.oregonstate.edu")
    try:
        r2 = AIMRobot(loop=loop, host=ip2)
    except Exception as e:
        print("Failed to initialize Robot 2:", e)
        return
    
    # We must assign the global `robot_for_loading` so FSM instantiation hooks up properly
    evbase.robot_for_loading = r1
    program.robot_for_loading = r1
    prog1 = Robot1Program()
    
    evbase.robot_for_loading = r2
    program.robot_for_loading = r2
    prog2 = Robot2Program()
    
    # Start both FSMs asynchronously
    r1.loop.call_soon_threadsafe(prog1.start)
    r2.loop.call_soon_threadsafe(prog2.start)
    
    print("Both State Machines started. Shared memory is live.")
    print("Press Ctrl+C to stop.")
    
    try:
        while True:
            time.sleep(2)
            print("--- Shared Memory Status ---")
            print(f"Barrels Seen by R1: {ScoreKeeper.barrel_count}")
            print(f"Hit Score => R1: {ScoreKeeper.score1} | R2: {ScoreKeeper.score2}")
            if ScoreKeeper.r1_pose:
                print(f"  R1 Pose: {ScoreKeeper.r1_pose.x:.1f}, {ScoreKeeper.r1_pose.y:.1f}")
            if ScoreKeeper.r2_pose:
                print(f"  R2 Pose: {ScoreKeeper.r2_pose.x:.1f}, {ScoreKeeper.r2_pose.y:.1f}")
    except KeyboardInterrupt:
        print("Exiting and stopping loops...")
        os._exit(0)

if __name__ == '__main__':
    main()
