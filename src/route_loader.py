import json
import os
import argparse
import argparse
import typing
from .pose import Pose


def assert_key_exists(key: str, data_dict: dict):
    assert key in data_dict, "{} is required".format(key)


class Map:
    def __init__(name: str, map_dict: dict):
        self.name = name
        self.next_map: str | None = map_dict["next_map"]
        self.description: str = map_dict["description"]
        self.waypoint_poses = []

        for waypoint_dict in map_dict["route"]:
            pose = Pose(waypoint_dict["name"], *waypoint_dict["pose"])
            self.waypoint_poses.append(pose)

        self.current_waypoint_idx = -1

    def move_to_next_waypoint(self):
        """Gets the next waypoint pose to move to

        Returns:
            Pose: a Pose object with the next waypoint pose.
        """
        if self.is_complete():
            print("Reached the end of the map: {}".format(self.name))
            assert (
                False
            ), "Map already complete. You should be checking is_route_complete() before moving to next waypoint."

        # We start off with current_waypoint_idx = -1, so this is okay
        self.current_waypoint_idx += 1
        return self.waypoint_poses[self.current_waypoint_idx]

    def is_complete(self):
        """Checks if all the waypoints for this Map have been reached

        Returns:
            bool: whether all the waypoints for this map have been reached
        """
        return self.current_waypoint_idx == len(self.route) - 1

    def is_final_map(self):
        """Checks to see if this map is the final map of the route

        Returns:
            bool: True if next_map is None. False otherwise.
        """
        return self.next_map == None

    @classmethod
    def validate_map(cls, map_dict: dict):
        # The next_map should be a str or None (if last map)
        assert map_dict["next_map"] is None or isinstance(map_dict["next_map"], str)
        assert_key_exists("description", map_dict)
        assert_key_exists("route", map_dict)
        Map.validate_map_route(map_dict["route"])

    @classmethod
    def validate_map_route(cls, route_list: list):
        assert isinstance(route_list, list)

        for waypoint_dict in route_list:
            assert isinstance(waypoint_dict, dict)
            assert_key_exists("name", waypoint_dict)
            assert_key_exists("description", waypoint_dict)
            assert_key_exists("pose", waypoint_dict)

            assert isinstance(waypoint_dict["name"], str)
            assert isinstance(waypoint_dict["description"], str)
            assert isinstance(waypoint_dict["pose"], list)

            assert (
                len(waypoint_dict["pose"]) == 7
            ), "Waypoint {} pose should have 7 values".format(waypoint_dict["name"])


class Route:
    """A class to handle interacting with Routes"""

    def __init__(route_json_path: str):
        """Create a new Route instance

        Args:
            route_json_path (str): the path to the route JSON configuration
        """
        Route.validate_route_json(route_json_path)

        # We can now assume that the map JSON is valid

        self.maps = {}
        self.start_map_key = route_data["start_map"]
        self.current_map: Map = None
        self.load_route(route_json_path)

        for map_name, map_dict in route_data["maps"].items():
            self.maps[map_name] = Map(map_name, map_dict)
        
        self.current_map = self.maps[self.start_map_key]

    def get_next_waypoint(self):
        if self.current_map.is_complete():
            # If we finished the map

            if self.current_map.is_final_map():
                # If this map is the final map, the path is complete
                print("Finished the final map")
                return None
            
            # Update the map
            self.current_map = self.maps[self.current_map.next_map]
    
        return self.current_map.get_next_waypoint()

    def is_complete(self):
        return self.current_map.

    @classmethod
    def validate_route_json(cls, route_json_path: str):
        assert os.path.isfile(route_json_path), "Route JSON not found: {}".format(
            route_json_path
        )

        with open(route_json_path) as f:
            route_data = json.load(f)

        assert_key_exists("start_map", route_data)
        assert_key_exists("maps", route_data)

        assert isinstance(
            route_data["maps"], dict
        ), "The maps key should contain a dictionary of maps"

        for map_key, map_dict in route_data["maps"].items():
            Map.validate_map(map_dict)


def main(route_json_path: str, validate: bool):
    if validate:
        Route.validate_route_json(route_json_path)
        print("The route is correctly configured")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-r", "--route", type=str, help="The path to the route JSON")
    ap.add_argument(
        "-v",
        "--validate",
        action="store_true",
        help="Only validate the route JSON (default to False)",
    )
    args = vars(ap.parse_args())
    main(args["route"], args["validate"])
