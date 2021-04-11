import json
import os
import argparse
import argparse
from pose import Pose
import yaml


def assert_key_exists(key, data_dict):
    """[summary]

    Args:
        key (str): [description]
        data_dict (dict): [description]
    """
    assert key in data_dict, "{} is required".format(key)


def is_number(x):
    return isinstance(x, int) or isinstance(x, float)


class Map:
    def __init__(self, name, map_dict, default_upper_lidar_threshold):
        """[summary]

        Args:
            name (str): [description]
            map_dict (dict): [description]
            default_upper_lidar_threshold (float): default upper range for lidar scan
        """
        self.name = name

        self.upper_lidar_threshold = default_upper_lidar_threshold
        if "upper_scan_threshold" in map_dict:
            self.upper_lidar_threshold = map_dict["upper_scan_threshold"]

        # str | None
        self.next_map = map_dict["next_map"]

        # str | None
        self.description = map_dict["description"]
        self.waypoint_poses = []

        for waypoint_dict in map_dict["route"]:
            # Skip over any waypoints marked with "skip": true
            if "skip" in waypoint_dict and waypoint_dict["skip"]:
                continue

            pose_specific_lidar_threshold = self.upper_lidar_threshold
            if "upper_scan_threshold" in waypoint_dict:
                pose_specific_lidar_threshold = waypoint_dict["upper_scan_threshold"]

            pose = Pose.pose_from_xy_euler(
                waypoint_dict["name"],
                waypoint_dict["x"],
                waypoint_dict["y"],
                waypoint_dict["euler_angle"],
            )
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
        return self.current_waypoint_idx == len(self.waypoint_poses) - 1

    def is_final_map(self):
        """Checks to see if this map is the final map of the route

        Returns:
            bool: True if next_map is None. False otherwise.
        """
        return self.next_map is None

    @staticmethod
    def validate_map(map_dict):
        """[summary]

        Args:
            map_dict (dict): [description]
        """

        # The next_map should be a str or None (if last map)
        assert map_dict["next_map"] is None or isinstance(map_dict["next_map"], str)

        # If the upper threshold is specified, it must be a number
        assert "upper_scan_threshold" not in map_dict or is_number(
            map_dict["upper_scan_threshold"]
        )
        assert_key_exists("description", map_dict)
        assert_key_exists("route", map_dict)
        Map.validate_map_route(map_dict["route"])

    @staticmethod
    def validate_map_route(route_list):
        """[summary]

        Args:
            route_list (list): [description]
        """
        assert isinstance(route_list, list)

        for waypoint_dict in route_list:
            assert isinstance(waypoint_dict, dict)
            assert_key_exists("name", waypoint_dict)
            assert_key_exists("description", waypoint_dict)
            assert_key_exists("x", waypoint_dict)
            assert_key_exists("y", waypoint_dict)
            assert_key_exists("euler_angle", waypoint_dict)

            assert isinstance(waypoint_dict["name"], str)
            assert isinstance(waypoint_dict["description"], str)
            assert is_number(waypoint_dict["x"])
            assert is_number(waypoint_dict["y"])
            assert is_number(waypoint_dict["euler_angle"])

            # If the upper threshold is specified, it must be a number
            assert "upper_scan_threshold" not in waypoint_dict or is_number(
                waypoint_dict["upper_scan_threshold"]
            )


class Route:
    """A class to handle interacting with Routes"""

    def __init__(
        self, route_json_path, default_upper_lidar_threshold=15.0, dry_run=False
    ):
        """Create a new Route instance

        Args:
            route_json_path (str): the path to the route JSON configuration
            default_upper_lidar_threshold (float): default upper range for lidar scan
            dry_run (bool): if true, won't send commands to ARK
        """
        self.dry_run = dry_run
        route_data = Route.validate_route_json(route_json_path)

        # We can now assume that the map JSON is valid

        self.maps = {}
        self.start_map_key = route_data["start_map"]

        # Map type
        self.current_map = None

        for map_name, map_dict in route_data["maps"].items():
            self.maps[map_name] = Map(
                map_name,
                map_dict,
                default_upper_lidar_threshold=default_upper_lidar_threshold,
            )

        self.current_map = self.maps[self.start_map_key]

        if self.dry_run:
            return

        from ark_interface import ARK
        import rospy

        # Load the initial map
        success = ARK.load_map(self.current_map.name)

        if success:
            rospy.loginfo("Initial map loaded: {}".format(self.current_map.name))
        else:
            rospy.logfatal("Unable to load initial map")

    def load_next_map(self):
        # Update the map
        self.current_map = self.maps[self.current_map.next_map]

        if self.dry_run:
            return

        from ark_interface import ARK
        import rospy

        success = ARK.load_map(self.current_map.name)

        if not success:
            rospy.logerr(
                "Tried to load map {} and timed out".format(self.current_map.name)
            )

    def get_next_waypoint(self):
        if self.current_map.is_complete():
            # If we finished the map

            if self.current_map.is_final_map():
                # If this map is the final map, the path is complete
                print("Finished the final map: {}".format(self.current_map.name))
                return None

            self.load_next_map()

        return self.current_map.move_to_next_waypoint()

    def is_complete(self):
        return self.current_map.is_final_map() and self.current_map.is_complete()

    @staticmethod
    def validate_route_json(route_json_path):
        """[summary]

        Args:
            route_json_path (str): path to the JSON file for the route

        Returns:
            (dict): route data dictionary
        """
        assert os.path.isfile(route_json_path), "Route JSON not found: {}".format(
            route_json_path
        )

        with open(route_json_path) as f:
            # Need to use yaml instead of JSON to avoid issues with
            # strings being converted into unicode
            # See: https://stackoverflow.com/a/16373377/6942666
            route_data = yaml.safe_load(f)

        assert_key_exists("start_map", route_data)
        assert_key_exists("maps", route_data)

        assert isinstance(
            route_data["maps"], dict
        ), "The maps key should contain a dictionary of maps"

        for map_key, map_dict in route_data["maps"].items():
            Map.validate_map(map_dict)

        return route_data


def main(route_json_path, validate):
    """[summary]

    Args:
        route_json_path (str): [description]
        validate (bool): [description]
    """
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
