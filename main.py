import network
import argparse
import pandas as pd

if __name__=='__main__':
    parser = argparse.ArgumentParser()

    parser.add_argument('height', help='Height of warehouse map')
    parser.add_argument('width', help='Width of warehouse map')
    parser.add_argument('agents', help='csv file directory for agents')
    parser.add_argument('packages', help='csv file directory for packages')
    parser.add_argument('dir_path', help='csv file directory for output figures')

    args_dict = vars(parser.parse_args())
    height, width = int(args_dict['height']), int(args_dict['width'])
    dict_agents = pd.read_csv(args_dict['agents']).to_dict('records')
    dict_packages = pd.read_csv(args_dict['packages']).to_dict('records')

    sample = network.Warehousemap(height, width, dict_agents, dict_packages, args_dict['dir_path'])
    sample.route_planning()
    sample.run_simulation()