from map_generator import MapGenerator
import textwrap
from pathlib import Path

custom_lookup_table = {
    (True, False): False,
    (True, True): True,
    (False, True): True,
    (False, False): True,
}


def custom_lookup(a: bool, b: bool) -> bool:
    return custom_lookup_table[(a, b)]


custom_conf_equivalence = {
    "conf_amcl_kinect": "amcl-kinect",
    "conf_amcl_lidar": "amcl-lidar",
    "conf_mprt_kinect": "mrpt-kinect",
    "conf_mprt_lidar": "mrpt-kinect",
    "conf_aruco": "aruco",
    "conf_aruco_headlamp": "aruco",
}


def conf_equivalence(conf):
    return custom_conf_equivalence[conf]


class Conf:
    def __init__(self, name, energy, accuracy, dark, unsafe):
        self.name = name
        self.energy = energy
        self.accuracy = accuracy
        self.dark = dark
        self.unsafe = unsafe


class PrismModelgenerator(MapGenerator):
    def __init__(
            self,
            num_nodes=30,
            nodes_skip=0.1,
            unconnected_amount=0.15,
            unsafe_amount=0.25,
            dark_amount=0.25,
            max_battery=32560,
            min_battery=500):

        MapGenerator.__init__(
            self,
            num_nodes,
            nodes_skip,
            unconnected_amount,
            unsafe_amount,
            dark_amount)
        self.configurations = [
            Conf("conf_amcl_kinect", 17790.0, 1.0, False, True),
            Conf("conf_amcl_lidar", 19790.0, 0.7, False, False),
            Conf("conf_mprt_kinect", 18942.0, 0.9, False, True),
            Conf("conf_mprt_lidar", 20942.0, 0.6, False, False),
            Conf("conf_aruco", 16963.0, 0.8, False, True),
            Conf("conf_aruco_headlamp", 26963.0, 0.8, True, True)
        ]

        self.file_header = textwrap.dedent(f'''
        mdp

        const MAX_BATTERY={max_battery};
        const MIN_BATTERY={min_battery};
        const INITIAL_BATTERY;
        const INITIAL_LOCATION;
        const TARGET_LOCATION;
        const INITIAL_CONFIGURATION;\n
        ''')

    def generate_prism_model(
            self,
            file_path,
            nav_path,
            with_probabilities=True):
        consts = ""
        for i in range(len(self.configurations)):
            consts += f"const {self.configurations[i].name}={i};\n"

        consts += "\n"
        for i in range(len(nav_path)):
            consts += f"const l{nav_path[i]}={i};\n"

        formula_distances = "\n"
        formula_bat_update = "\n"
        formula_p_collide = "\n"
        speed = 0.68  # m/s
        for i in range(len(nav_path) - 1):
            distance = self.graph[nav_path[i]][nav_path[i + 1]]['weight']
            formula_distances += f"formula dist_l{nav_path[i]}_l{nav_path[i+1]}={distance};\n"

            # energy = speed*
            formula_bat_update_ = f"formula b_upd_l{nav_path[i]}_l{nav_path[i+1]}=\n"
            for conf in self.configurations:
                energy = int(distance * speed * conf.energy * 0.01)
                formula_bat_update_ += f"\tc={conf.name}? max(0, b-{energy}) :\n"
                hitrate = 0.0
                if with_probabilities is True:
                    conf_name_ = conf_equivalence(conf.name)
                    try:
                        hitrate = self.graph[nav_path[i]][nav_path[i + 1]
                                                          ]['hitrate'][conf_name_]['hitrate']
                    except KeyError:
                        hitrate = 0.0
                formula_p_collide += f"formula p_col_{conf.name}_l{nav_path[i]}_l{nav_path[i+1]}={hitrate};\n"
            formula_bat_update_ += " \t0;\n"
            formula_bat_update += formula_bat_update_

        robot_module = textwrap.dedent(f'''
        module robot

        b:[0..MAX_BATTERY] init INITIAL_BATTERY;
        l:[0..{len(nav_path)-1}] init INITIAL_LOCATION;
        c:[0..{len(self.configurations)-1}] init INITIAL_CONFIGURATION;

        interrupted: bool init false;
        goal: bool init false;

        rd: bool init false; collided: bool init false;

        [interrupt] true -> (interrupted'=true);
        [finish] (target_location) & (!goal) -> (goal'=true);
        ''')

        reconfig_actions = "\n"
        for conf in self.configurations:
            reconfig_actions += f"[t_set_{conf.name}] (c!={conf.name}) & (!rd) &(!stop) -> (c'={conf.name}) & (rd'=true);\n"

        move_actions = "\n"
        for i in range(len(nav_path) - 1):
            for conf in self.configurations:
                move_action = ""
                if custom_lookup(
                        self.graph[nav_path[i]][nav_path[i + 1]]['dark'], conf.dark):
                    move_action = f"[l{nav_path[i]}_l{nav_path[i+1]}] (l=l{nav_path[i]}) & (!stop) & (c={conf.name}) ->"
                    move_action += f"\n\tp_col_{conf.name}_l{nav_path[i]}_l{nav_path[i+1]}: (l'=l{nav_path[i+1]}) & (b'=b_upd_l{nav_path[i]}_l{nav_path[i+1]}) & (collided'=true) & (rd'=false)"
                    move_action += f"\n\t+ 1-(p_col_{conf.name}_l{nav_path[i]}_l{nav_path[i+1]}): (l'=l{nav_path[i+1]}) & (b'=b_upd_l{nav_path[i]}_l{nav_path[i+1]}) & (collided'=false) & (rd'=false);\n"
                move_actions += move_action
            move_actions += "\n"

        energy_reward = textwrap.dedent(f'''
            rewards "energy"
                [finish] true : b;
            endrewards
            ''')

        with open(file_path, "w") as file:
            file.write(self.file_header)
            file.write(consts)
            # file.write(formula_distances)
            file.write(formula_bat_update)
            file.write(formula_p_collide)
            file.write("\nformula target_location = l=TARGET_LOCATION;\n")
            file.write("formula stop = goal | b<MIN_BATTERY | interrupted;\n")
            file.write(robot_module)
            file.write(reconfig_actions)
            file.write(move_actions)
            file.write("\nendmodule\n")
            file.write(energy_reward)


if __name__ == '__main__':
    prism_generator = PrismModelgenerator()
    base_folder = Path("map_camara_2020_paper")
    prism_generator.load_json(base_folder / "map-p2cp3.json")
    nav_path = prism_generator.find_shortest_path(from_node=1, to_node=18)
    print(nav_path)
    prism_generator.generate_prism_model("prism/test.prism", nav_path)
