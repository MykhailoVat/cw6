from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from controllers.system_controller import SystemController
from opt.error import RobotError, SystemControlError


def wait_for_reset(client):
    print("[main.py]: Press CTRL + C to reset and start robot.")

    try:
        while True:
            client.step()
    except KeyboardInterrupt:
        print("[main.py]: Reset signal received.")


def main():
    client = RemoteAPIClient()
    sim = client.require('sim')
    sim_ik = client.require('simIK')

    system = SystemController(sim, sim_ik)
    system.start()

    client.setStepping(True)
    sim.startSimulation()

    try:
        while True:
            try:
                system.update()
                client.step()

            except (RobotError, SystemControlError):
                wait_for_reset(client)
                system.reset()

    except KeyboardInterrupt:
        print('\n[main.py]: Stopping simulation...')

    finally:
        cleanup_client = None
        stop_client = None

        try:
            cleanup_client = RemoteAPIClient()
            cleanup_sim = cleanup_client.require('sim')
            cleanup_ik = cleanup_client.require('simIK')

            system.cleanup(cleanup_sim, cleanup_ik)
            print('[main.py]: System cleanup completed.')

        except Exception as e:
            print(f'[main.py]: System cleanup error: {e}')

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
