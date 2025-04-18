import carla

from simulation import config


def main():
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(20.0)

    sim_world = client.load_world(config.SIM_WORLD)
    
if __name__ == '__main__':
    main()
