"""Example script of moving robot joint positions."""
import argparse
import pickle
import threading
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from deoxys import config_root
from deoxys.franka_interface import FrankaInterface
from deoxys.utils import YamlConfig
from deoxys.utils.input_utils import input2action
from deoxys.utils.log_utils import get_deoxys_example_logger

logger = get_deoxys_example_logger()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface-cfg", type=str, default="charmander.yml")
    parser.add_argument(
        "--controller-cfg", type=str, default="joint-position-controller.yml"
    )
    args = parser.parse_args()
    return args


def main():
    args = parse_args()

    robot_interface = FrankaInterface(
        config_root + f"/{args.interface_cfg}", use_visualizer=False
    )
    controller_cfg = YamlConfig(
        config_root + f"/{args.controller_cfg}").as_easydict()

    controller_type = "JOINT_POSITION"

    print("MOVING TO START STATE")

    reset_joint_positions = np.array([
        0.09162008114028396,
        -0.1,
        -0.01990020486871322,
        -2.4732269941140346,
        -0.01307073642274261,
        2.30396583422025,
        0.8480939705504309,
    ])
    action = np.append(reset_joint_positions, [-1.0])

    while robot_interface.state_buffer_size == 0:
        logger.warn("Robot state not received")
        time.sleep(0.5)

    while True:
        logger.info(
            f"Current Robot joint: {np.round(robot_interface.last_q, 3)}")
        logger.info(
            f"Desired Robot joint: {np.round(robot_interface.last_q_d, 3)}")

        if (
            np.max(
                np.abs(
                    np.array(robot_interface._state_buffer[-1].q)
                    - np.array(reset_joint_positions)
                )
            )
            < 1e-3
        ):
            break
        robot_interface.control(
            controller_type=controller_type,
            action=action,
            controller_cfg=controller_cfg,
        )

    print("MOVING ALONG A PATH")

    while robot_interface.state_buffer_size == 0:
        logger.warn("Robot state not received")
        time.sleep(0.5)

    start = time.time()
    timeout = start + 3
    inc_idx = 0
    joint_positions = reset_joint_positions

    while time.time() < timeout:
        logger.info(
            f"Current Robot joint: {np.round(robot_interface.last_q, 3)}")
        logger.info(
            f"Desired Robot joint: {np.round(robot_interface.last_q_d, 3)}")

        inc_times = np.array([0., 1., 2., 3., 4.])
        if time.time() - start > inc_times[inc_idx]:
            delta = -0.03  # For example, 0.01 radians
            joint_index = 2
            joint_positions[joint_index] += delta

            inc_idx = inc_idx+1

        action = np.append(joint_positions, [-1.0])

        print("action\n", action)

        robot_interface.control(
            controller_type=controller_type,
            action=action,
            controller_cfg=controller_cfg,
        )

    robot_interface.close()


if __name__ == "__main__":
    main()
