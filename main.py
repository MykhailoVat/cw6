import threading
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from controllers.conveyor import Conveyor
from controllers.robot import Robot
from opt.error import RobotError

def wait_for_reset():
    print("[main.py]: Press CTRL + C to reset and start robot.")

    try:
        while True:
            threading.Event().wait()
    except KeyboardInterrupt:
        print("[main.py]: Reset signal received.")

def main():
    client = RemoteAPIClient()
    sim = client.require('sim')
    sim_ik = client.require('simIK')

    robot = Robot(sim, sim_ik)
    robot.start()

    conveyor = Conveyor(sim)
    conveyor.start()

    client.setStepping(False)
    sim.startSimulation()

    try:
        while True:
            try:
                robot.update()
                conveyor.update()
                #client.step()

            except RobotError as e:
                wait_for_reset()
                robot.reset()
                robot.start()

    except KeyboardInterrupt:
        print('\n[main.py]: Stopping simulation...')

    finally:
        cleanup_client = None
        stop_client = None

        try:
            cleanup_client = RemoteAPIClient()
            cleanup_sim = cleanup_client.require('sim')
            cleanup_ik = cleanup_client.require('simIK')

            robot.cleanup(cleanup_sim, cleanup_ik)
            print('[main.py]: Robot cleanup completed.')

        except Exception as e:
            print(f'[main.py]: Robot cleanup error: {e}')

        finally:
            if cleanup_client:
                cleanup_client.socket.close()

        try:
            stop_client = RemoteAPIClient()
            stop_sim = stop_client.require('sim')

            stop_sim.stopSimulation()
            print('[main.py]: Simulation stopped.')

        except Exception as e:
            print(f'[main.py]: Stop simulation error: {e}')

        finally:
            if stop_client:
                stop_client.socket.close()

        client.socket.close()

if __name__ == '__main__':
    main()
